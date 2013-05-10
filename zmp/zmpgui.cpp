#include "hubo-zmp.h"
#include <HuboPlus.h>
#include <math.h>
#include <mzcommon/MzGlutApp.h>
#include <mzcommon/TimeUtil.h>
#include <getopt.h>
// for keyboard interrupt
#include <termio.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>

#include "zmpwalkgenerator.h"
#include "footprint.h"

using namespace fakerave;

typedef std::vector< zmp_traj_element_t > TrajVector;

size_t seconds_to_ticks(double s) {
  return size_t(round(s*TRAJ_FREQ_HZ));
}
/*TODO remove this
*/
const int stance_foot_table[4] = { 0, 1, 0, 1 };
const int swing_foot_table[4] = { -1, -1, 1, 0 };

const stance_t next_stance_table[4] = {
  SINGLE_LEFT,
  SINGLE_RIGHT,
  DOUBLE_RIGHT,
  DOUBLE_LEFT
};


class ZmpDemo: public MzGlutApp {
public:

  HuboPlus& hplus;
  KinBody& kbody;

  HuboPlus::KState state;
  vec3 forces[2];
  vec3 torques[2];
  vec3 actualCom;
  vec3 actualComVel;
  vec3 actualComAcc;

  Transform3Array xforms;

  const TrajVector& traj;
  
  size_t cur_index;

  int stance_foot;
  Transform3 stance_foot_xform;
  
  GLUquadric* quadric;

  bool animating;

  ZmpDemo(int argc, char** argv, HuboPlus& h, const TrajVector &t):
    MzGlutApp(argc, argv, GLUT_DOUBLE | GLUT_DEPTH | GLUT_RGB | GLUT_MULTISAMPLE),
    hplus(h),
    kbody(h.kbody),
    traj(t),
    cur_index(-1)

  {

    initWindowSize(640, 480);
    createWindow("Hubo Demo");
    setupBasicLight(vec4f(1,1,1,0));

    double bz = 0.85;

    camera.aim(vec3f(3, 0, bz),
               vec3f(0, 0, bz),
               vec3f(0, 0, 1));

    camera.setPerspective();

    camera.setRotateType(GlCamera::ROTATE_2_AXIS);

    camera.setHomePosition();

    quadric = gluNewQuadric();

    animating = false;

    kbody.compileDisplayLists();

    state.jvalues.resize(kbody.joints.size());

    setTimer(40, 0);
    
    resetCurrent();

  }

  void resetCurrent() {

    cur_index = 0; 
    stance_foot_xform = Transform3();
    stance_foot = stance_foot_table[traj[0].stance];
    setStateFromTraj(traj[0]);


  }
    
  
  
  void setStateFromTraj(const zmp_traj_element_t& cur) {
    
    for (size_t hi=0; hi<hplus.huboJointOrder.size(); ++hi) {
      size_t ji = hplus.huboJointOrder[hi];
      if (ji != size_t(-1)) {
      	state.jvalues[ji] = cur.angles[hi];
      }
    }
    
    kbody.transforms(state.jvalues, xforms);

    Transform3 foot_fk = kbody.manipulatorFK(xforms, stance_foot);
    state.setXform(stance_foot_xform * foot_fk.inverse());
    
    for (int a=0; a<3; ++a) {
      actualCom[a] = cur.com[a][0];
      actualComVel[a] = cur.com[a][1];
      actualComAcc[a] = cur.com[a][2];
      for (int f=0; f<2; ++f) {
	forces[1-f][a] = cur.forces[f][a];
	torques[1-f][a] = cur.torque[f][a]; // TODO: FIXME: pluralization WTF?
      }
    }

//    std::cerr << "current index: " << cur_index << "/" << traj.size() << ", stance=" << cur.stance << "\n";  
    
  }

  virtual void deltaCurrent(int delta) {

    int dmin = -cur_index;
    int dmax = traj.size()-1-cur_index;
    delta = std::max(dmin, std::min(delta, dmax));
    if (!delta) { return; }

    int dd = delta < 0 ? -1 : 1;
    delta *= dd;
    assert(delta > 0);

    for (int i=0; i<delta; ++i) {

      cur_index += dd;

      // see if the stance foot has swapped
      int new_stance_foot = stance_foot_table[traj[cur_index].stance];

      if (new_stance_foot != stance_foot) {
        stance_foot = new_stance_foot;
        stance_foot_xform = state.xform() * kbody.manipulatorFK(xforms, stance_foot);
      }

      setStateFromTraj(traj[cur_index]);

    }

  }

  virtual void display() {

    MzGlutApp::display();

    glMatrixMode(GL_MODELVIEW);

    glColor3ub(0, 127, 127);
    glNormal3f(0, 0, 1);
    glBegin(GL_LINES);
    double s = 0.5;
    double n = 10;
    double x = s*n;
    for (int i=-n; i<=n; ++i) {
      glVertex2d(x, i*s);
      glVertex2d(-x, i*s);
      glVertex2d(i*s, x);
      glVertex2d(i*s, -x);
    }
    glEnd();

    glPushMatrix();
    glstuff::mult_transform(state.xform());

    hplus.render(xforms);
    glClear(GL_DEPTH_BUFFER_BIT);

    hplus.kbody.renderSkeleton(xforms, quadric);

    // Force and torque arrows
    for (int f=0; f<2; ++f) {
      Transform3 fk = hplus.kbody.manipulatorFK(xforms, f);
      glPushMatrix();
      glstuff::mult_transform(fk);
      glTranslated(0, 0, hplus.footAnkleDist);
      double fscl = 1.0/400;
      glColor3ub(255,255,0);
      glstuff::draw_arrow(quadric, vec3(0), fscl*forces[f], 0.02);
      glColor3ub(255,128,0);
      glstuff::draw_arrow(quadric, vec3(0), fscl*torques[f], 0.02);

      glPopMatrix();
    }

    glPopMatrix();


    glPushMatrix();
    glstuff::mult_transform(stance_foot_xform);

    // CoM sphere
    glColor3ub(255, 0, 255);
    glPushMatrix();
    glTranslated(actualCom[0], actualCom[1], actualCom[2]+hplus.footAnkleDist);
    gluSphere(quadric, 0.05, 32, 24);
    glPopMatrix();

    // CoM projection disk
    glPushMatrix();
    glTranslated(actualCom[0], actualCom[1], 0.01);
    glColor3ub(127, 0, 127);
    gluDisk(quadric, 0, 0.05, 32, 1);
    glRotated(180, 1, 0, 0);
    gluDisk(quadric, 0, 0.05, 32, 1);
    glPopMatrix();

    // Acceleration arrow
    double fscl = 1.0/2.0;
    glPushMatrix();
    glTranslated(actualCom[0], actualCom[1], actualCom[2]+hplus.footAnkleDist);
    glColor3ub(0, 255, 0);
    glstuff::draw_arrow(quadric, vec3(0), fscl*actualComAcc, 0.02);
    glPopMatrix();
    
    
    glPopMatrix();

    glutSwapBuffers();

  }

  virtual void timer(int value) {

    if (animating) {
      if (cur_index+1 < traj.size()) { 
	deltaCurrent(5);
	glutPostRedisplay();
      } else {
	animating = false;
      }
    }

    setTimer(40, 0);    

  }

  virtual void keyboard(unsigned char key, int x, int y) {
    switch (key) {
    case '-':
      animating = false;
      deltaCurrent(-1);
      glutPostRedisplay();
      break;
    case '+':
    case '=':
      animating = false;
      deltaCurrent(1);
      glutPostRedisplay();
      break;
    case 'r':
      animating = false;
      resetCurrent();
      glutPostRedisplay();
      break;
    case '\r':
    case '\n':
      animating = !animating;
      break;
    default: 
      MzGlutApp::keyboard(key, x, y);
    }


    
  }

};

/**
* @function: validateCOMTraj(Eigen::MatrixXd& comX, Eigen::MatrixXd& comY) 
* @brief: validation of COM output trajectory data
* @return: void
*/
void validateCOMTraj(Eigen::MatrixXd& comX, Eigen::MatrixXd& comY) {
    const double dt = 1.0/TRAJ_FREQ_HZ;
    double comVel, comAcc;
    Eigen::Matrix3d comStateDiffs;
    double comStateMaxes[] = {0.0, 0.0, 0.0};
    const double comStateTol[] = {2.0, 5.0}; // m/s, m/s^2, m/s^3
    for (int n=0; n<(comX.rows()-1); n++) {
      // Calculate COM vel and acc norms from x and y components
      comVel = sqrt((comX(n+1,0)-comX(n,0))*(comX(n+1,0)-comX(n,0)) + (comY(n+1,0)-comY(n,0))*(comY(n+1,0)-comY(n,0)))/dt;
      comAcc = sqrt((comX(n+1,1)-comX(n,1))*(comX(n+1,1)-comX(n,1)) + (comY(n+1,1)-comY(n,1))*(comY(n+1,1)-comY(n,1)))/dt;
      // Update max state values
      if (comVel > comStateMaxes[0]) comStateMaxes[0] = comVel;
      if (comAcc > comStateMaxes[1]) comStateMaxes[1] = comAcc;
      // Check if any are over limit
      if (comVel > comStateTol[0]) {
          std::cerr << "COM velocity sample " << n+1 << "is larger than " << comStateTol[0] << "(" << comVel << ")\n";
      }
      if (comAcc > comStateTol[1]) {
          std::cerr << "COM acceleration of sample " << n+1 << "is larger than " << comStateTol[1] << "(" << comAcc << ")\n";
      }
    }
    std::cerr << "comMaxVel: " << comStateMaxes[0]
              << "\ncomMaxAcc: " << comStateMaxes[1] << std::endl;


}
  
/**
* @function: validateOutputData(TrajVector& traj)
* @brief: validation of joint angle output trajectory data
* @return: void
*/
void validateOutputData(TrajVector& traj) {
    const double dt = 1.0/TRAJ_FREQ_HZ;
    double maxJointVel=0;
    double jointVel;
    const double jointVelTol = 6.0; // radians/s
    for (int n=0; n<(int)(traj.size()-1); n++) {
      for (int j=0; j<HUBO_JOINT_COUNT; j++) {  
        jointVel = (traj[n+1].angles[j] - traj[n].angles[j])/dt;
        if (jointVel > jointVelTol) {
          std::cerr << "change in joint " << j << "is larger than " << jointVelTol << "(" << jointVel << ")\n";
        }
        if (jointVel > maxJointVel) maxJointVel = jointVel;
      }
    }
    std::cerr << "maxJntVel: " << maxJointVel << std::endl;
}


int main(int argc, char** argv)
{
  // load in openRave hubo model
  HuboPlus hplus("../myhubo.kinbody.xml");

  ach_channel_t zmp_chan;
  ach_open( &zmp_chan, HUBO_CHAN_ZMP_TRAJ_NAME, NULL );

  zmp_traj_t curTrajectory, nextTrajectory;
  memset( &curTrajectory, 0, sizeof(curTrajectory) );
  memset( &nextTrajectory, 0, sizeof(nextTrajectory) );

  zmpgui_traj_t curGuiTrajectory, nextGuiTrajectory;
  memset( &curGuiTrajectory, 0, sizeof(curGuiTrajectory) );
  memset( &nextGuiTrajectory, 0, sizeof(nextGuiTrajectory) );
  size_t fs;

  std::cout << "Getting trajectory from ach\n";
  std::cout << "count: " << curTrajectory.count << std::endl;
  //while(curTrajectory.count <= 0)
  ach_get( &zmp_chan, &curTrajectory, sizeof(curTrajectory), &fs, NULL, ACH_O_WAIT );

  std::cout << "Got trajectory!\n";

  // convert c-array into std::vector
  int N = sizeof(curTrajectory.traj)/sizeof(curTrajectory.traj[0]);
  for(int i=0; i<N; i++)
    curGuiTrajectory.traj.push_back(curTrajectory.traj[i]);

  std::cout << "constructing demo\n";
  ZmpDemo demo(argc, argv, hplus, curGuiTrajectory.traj);

  std::cout << "running demo\n";
  demo.run();

  return 0;
}

// Local Variables:
// c-basic-offset: 2
// End:
//  bool nextTrajReady = false;
//  bool useNextTraj = false;
//  walkState_t walkState = STOP;
//  walkTransition_t walkTransition = STAY_STILL;

  // if we don't have a new trajectory or we're told to stop,
  // then keep checking for walk trajectory
//  while(curTrajectory.count <= 0 || walkState == STOP)
//  {
//      memset( &curTrajectory, 0, sizeof(curTrajectory) );
//      ach_get( &zmp_chan, &curTrajectory, sizeof(curTrajectory), &fs, NULL, ACH_O_LAST );
//  }

  // once we get a trajectory, check if we need to stabelize first or if we
  // can just keep walking, and then execute walk trajectory if not stopped
//  if( curTrajectory.walkTransition == SWITCH_WALK )
//  {
//  }


