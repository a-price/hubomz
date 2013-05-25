#include "hubo-zmp.h"
#include <HuboPlus.h>
#include <math.h>
#include <mzcommon/MzGlutApp.h>
#include <mzcommon/TimeUtil.h>
#include <getopt.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>

#include "zmpwalkgenerator.h"
#include "footprint.h"

using namespace fakerave;

typedef std::vector< zmp_traj_element_t > TrajVector;

size_t seconds_to_ticks(double s) {
  return size_t(round(s*ZMP_TRAJ_FREQ_HZ));
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

  ach_channel_t zmp_chan;
  zmp_traj_t nextTrajectory, curTrajectory;
  TrajVector curGuiTrajectory;
  bool nextTrajReady;
  bool useNextTraj;
  size_t trajNumber;

  HuboPlus::KState state;
  vec3 forces[2];
  vec3 torques[2];
  vec3 actualCom;
  vec3 actualComVel;
  vec3 actualComAcc;

  Transform3Array xforms;

  size_t cur_index;

  int stance_foot;
  Transform3 stance_foot_xform;
  
  GLUquadric* quadric;

  bool animating;

  ZmpDemo(int argc, char** argv, HuboPlus &h):
    MzGlutApp(argc, argv, GLUT_DOUBLE | GLUT_DEPTH | GLUT_RGB | GLUT_MULTISAMPLE),
    hplus(h),
    kbody(h.kbody),
    cur_index(-1)

  {
    ach_status r = ach_open( &zmp_chan, HUBO_CHAN_ZMP_TRAJ_NAME, NULL );
    fprintf(stdout, "AchOpen: %s \n", ach_result_to_string(r));

    trajNumber = 0;
    checkForNewTraj();
    nextTrajReady = false;
    useNextTraj = false;

    initWindowSize(640, 480);
    createWindow("Hubo ZMP Walking");
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

  /**
  * @function: validateOutputData(TrajVector& traj)
  * @brief: validation of joint angle output trajectory data
  * @return: void
  */
  bool validateTrajSwapIn(double prevAngles[HUBO_JOINT_COUNT], double newAngles[HUBO_JOINT_COUNT]) {
      const double dt = 1.0/ZMP_TRAJ_FREQ_HZ;
      double maxJointVel=0;
      double jointVel;
      bool OK = true;
      const double jointVelTol = 6.0; // radians/s
      for (int j=0; j<HUBO_JOINT_COUNT; j++) {   
        jointVel = (prevAngles[j] - newAngles[j])/dt;
        if (jointVel > jointVelTol) {
          std::cerr << "change in joint " << jointNames[j] << " is larger than " << jointVelTol << "(" << jointVel << ")\n";
          OK = false;
        }   
        if (jointVel > maxJointVel) maxJointVel = jointVel;
      }   
      std::cerr << "maxJntVel: " << maxJointVel << std::endl;
      return OK; 
  }

  /**
   * @function: convertToGuiTraj()
   * @brief: converts the traj in the struct from
   * a c-array of zmp_element_t to an std::vector
   * of zmp_element_t
  */
  virtual void convertToGuiTraj(zmp_traj_t &zmpTraj, TrajVector &zmpGuiTraj)
  {
    zmpGuiTraj.clear();
    for(size_t i=0; i<zmpTraj.count; i++)
      zmpGuiTraj.push_back(zmpTraj.traj[i]);
  }

  /**
   * @function: checkForNewTraj()
   * @brief: waits for new walk trajectory
  */
  virtual void checkForNewTraj()
  {
    size_t fs;
    std::cout << "Getting new trajectory\n";
    memset( &curTrajectory, 0, sizeof(curTrajectory) );
    while((curTrajectory.count <= 0 || curTrajectory.walkState == STOP) && curTrajectory.trajNumber <= trajNumber)
    {
      memset( &curTrajectory, 0, sizeof(curTrajectory) );
      ach_status r = ach_get( &zmp_chan, &curTrajectory, sizeof(curTrajectory), &fs, NULL, ACH_O_LAST );
    }
    std::cout << "Got Initial Traj # " << curTrajectory.trajNumber << std::endl;
    // convert new trajectory to gui trajectory
    convertToGuiTraj(curTrajectory, curGuiTrajectory);
    cur_index = 0;
    animating = true;
    nextTrajReady = false;
  }

  virtual void checkForNextTraj()
  {
    if(nextTrajReady == false)
    {
      // clear nextTrajectory contents
      memset( &nextTrajectory, 0, sizeof(nextTrajectory) );
      size_t fs;

      // get from channel
      ach_status r = ach_get( &zmp_chan, &nextTrajectory, sizeof(nextTrajectory), &fs, NULL, ACH_O_LAST );
      if((r == ACH_OK || r == ACH_MISSED_FRAME) && nextTrajectory.trajNumber > 0)
      {
        fprintf(stdout, "AchGet: %s \n", ach_result_to_string(r));
        std::cout << "Got trajectory # " << nextTrajectory.trajNumber << "\n";
      }

      if(nextTrajectory.walkTransition == WALK_TO_STOP)
      {
        useNextTraj = false;
        nextTrajReady = true;
      }

      // if we received a next trajectory
      if(nextTrajectory.trajNumber > curTrajectory.trajNumber)
      {
        // if start tick has been passed or switching walk type indicate it, otherwise indicate that we can use it
        if(cur_index > nextTrajectory.startTick || nextTrajectory.walkTransition == SWITCH_WALK)
        {
          std::cout << "Won't use next traj " << nextTrajectory.trajNumber << "\n";
          useNextTraj = false;
          nextTrajReady = true;
        }
        else
        {
          std::cout << "Going to use next traj # " << nextTrajectory.trajNumber << "\n";
          useNextTraj = true;
          nextTrajReady = true;
        }
      }
    }

    if(useNextTraj == true && nextTrajectory.startTick == cur_index)
    {
      bool OK;
      std::cout << "validating\n";
      OK = validateTrajSwapIn(nextTrajectory.traj[0].angles, curTrajectory.traj[cur_index].angles);
      if(OK == false)
      {
        std::cout << "Not swapping in next trajectory. Finishing current trajectory\n";
        nextTrajReady = true;
      }
      else
      {
        std::cout << "Swapping in next trajectory # " << nextTrajectory.trajNumber << ".\n";
        curTrajectory = nextTrajectory;
        convertToGuiTraj(curTrajectory, curGuiTrajectory);
        cur_index = 0;
        nextTrajReady = false;
      }
    }
  }

  // set state to initial trajectory tick joint values
  void resetCurrent() {

    cur_index = 0; 
    stance_foot_xform = Transform3();
    stance_foot = stance_foot_table[curGuiTrajectory[0].stance];
    setStateFromTraj(curGuiTrajectory[0]);
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
    int dmax = curGuiTrajectory.size()-1-cur_index;
    delta = std::max(dmin, std::min(delta, dmax));
    if (!delta) { return; }

    int dd = delta < 0 ? -1 : 1;
    delta *= dd;
    assert(delta > 0);
    for (int i=0; i<delta; ++i) {
      cur_index += dd;
      if(useNextTraj == false && nextTrajReady == true)
      {
        checkForNewTraj();
      }
      else
      {
        checkForNextTraj();
      }

      
      // see if the stance foot has swapped
      int new_stance_foot = stance_foot_table[curGuiTrajectory[cur_index].stance];

      if (new_stance_foot != stance_foot) {
        stance_foot = new_stance_foot;
        stance_foot_xform = state.xform() * kbody.manipulatorFK(xforms, stance_foot);
      }
      setStateFromTraj(curGuiTrajectory[cur_index]);
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
      if (cur_index+1 < curGuiTrajectory.size()) { 
	    deltaCurrent(5); // this set the display speed
	    glutPostRedisplay();
      } else {
        checkForNewTraj();
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
    const double dt = 1.0/ZMP_TRAJ_FREQ_HZ;
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
    const double dt = 1.0/ZMP_TRAJ_FREQ_HZ;
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

  ZmpDemo demo(argc, argv, hplus);

  std::cout << "running demo\n";
  demo.run();
  return 0;
}

// Local Variables:
// c-basic-offset: 2
// End:

/*
  ach_channel_t zmp_chan;
  ach_status r = ach_open( &zmp_chan, HUBO_CHAN_ZMP_TRAJ_NAME, NULL );
  fprintf(stdout, "%s \n", ach_result_to_string(r));

  zmp_traj_t curTrajectory;
  memset( &curTrajectory, 0, sizeof(curTrajectory) );

  zmpgui_traj_t curGuiTrajectory, nextGuiTrajectory;
  memset( &curGuiTrajectory, 0, sizeof(curGuiTrajectory) );
  size_t fs;

  std::cout << "Getting trajectory from ach\n";
  while(curTrajectory.count <= 0)
    r =  ach_get( &zmp_chan, &curTrajectory, sizeof(curTrajectory), &fs, NULL, ACH_O_WAIT );
  fprintf(stdout, "%s \n", ach_result_to_string(r));
  std::cout << "Got trajectory # " << curTrajectory.trajNumber << "! Count = " << curTrajectory.count << "\n";

  // convert c-array into std::vector
  int N = sizeof(curTrajectory.traj)/sizeof(curTrajectory.traj[0]);
  for(int i=0; i<N; i++)
    curGuiTrajectory.traj.push_back(curTrajectory.traj[i]);
*/

