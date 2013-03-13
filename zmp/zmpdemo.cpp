#include "hubo-zmp.h"
#include <HuboPlus.h>
#include <ZmpPreview.h>
#include <math.h>
#include <mzcommon/MzGlutApp.h>

using namespace fakerave;

typedef std::vector< zmp_traj_element_t > TrajVector;

size_t seconds_to_ticks(double s) {
  return size_t(round(s*TRAJ_FREQ_HZ));
}

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
  vec3 actualCom;

  Transform3Array xforms;

  const TrajVector& traj;
  
  size_t cur_index;

  int stance_foot;
  Transform3 stance_foot_xform;
  
  GLUquadric* quadric;

  bool animating;

  ZmpDemo(int argc, char** argv, HuboPlus& h, const TrajVector& t):
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
    }
			    

    std::cerr << "current index: " << cur_index << "/" << traj.size() << "\n";  
    
  }

  virtual void deltaCurrent(int delta) {

    int dmin = -cur_index;
    int dmax = traj.size()-1-cur_index;
    delta = std::max(dmin, std::min(delta, dmax));


    int dd = delta < 0 ? -1 : 1;

    for ( ; delta != 0; delta -= dd, cur_index += dd) {

      // see if the stance foot has swapped
      int new_stance_foot = stance_foot_table[traj[cur_index].stance];

      if (new_stance_foot != stance_foot) {
	stance_foot = new_stance_foot;
	stance_foot_xform = state.xform() * kbody.manipulatorFK(xforms, stance_foot);
      }

      setStateFromTraj(traj[cur_index]);

    }

    std::cerr << "current index: " << cur_index << "/" << traj.size() << "\n";

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

    glPopMatrix();


    glPushMatrix();
    glstuff::mult_transform(stance_foot_xform);

    glColor3ub(255, 0, 255);
    glPushMatrix();
    glTranslated(actualCom[0], actualCom[1], actualCom[2]);
    gluSphere(quadric, 0.05, 32, 24);
    glPopMatrix();

    glPushMatrix();
    glTranslated(actualCom[0], actualCom[1], 0.01);
    glColor3ub(127, 0, 127);
    gluDisk(quadric, 0, 0.05, 32, 1);
    glRotated(180, 1, 0, 0);
    gluDisk(quadric, 0, 0.05, 32, 1);
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
      deltaCurrent(-5);
      glutPostRedisplay();
      break;
    case '+':
    case '=':
      deltaCurrent(5);
      glutPostRedisplay();
      break;
    case 'r':
      resetCurrent();
      glutPostRedisplay();
    case '\r':
    case '\n':
      animating = !animating;
      break;
    default: 
      MzGlutApp::keyboard(key, x, y);
    }


    
  }

  



};


int main(int argc, char** argv) {

  if (argc < 2) {
    std::cout << "usage: " << argv[0] << " HUBOFILE.xml\n";
    return 1;
  }

  ach_channel_t zmp_chan;
  ach_open( &zmp_chan, HUBO_CHAN_ZMP_TRAJ_NAME, NULL );
  

  HuboPlus hplus(argv[1]);

  bool show_gui = true;

  double fy = hplus.defaultFootPos.y(); // half of horizontal separation distance between feet
  double sway = 0.085; // body sway (should be equal ish to fy)
//  double sway = 0.01;
  double fz = 0.05; // foot liftoff height

  double com_height = 0.58; // height of COM above ground
  double zmp_dt = 1.0/TRAJ_FREQ_HZ; // delta t for ZMP preview controller
  double zmp_R = 1e-10; // gain for ZMP controller
  size_t num_lookahead = seconds_to_ticks(2.5); // lookahead window size

  size_t startup_ticks = seconds_to_ticks(1.0); // hold ZMP at center for 1s at start
  size_t shutdown_ticks = seconds_to_ticks(1.0); // hold ZMP at center for 1s at end
  size_t double_support_ticks = seconds_to_ticks(0.05); // .05s of double support each step
  size_t single_support_ticks = seconds_to_ticks(0.70); // .70s of single support each step
  size_t n_steps = 8; // how many steps?

  size_t step_ticks = double_support_ticks + single_support_ticks;


  size_t total_ticks = startup_ticks + n_steps * step_ticks + shutdown_ticks;

  ZmpPreview preview(zmp_dt, com_height, num_lookahead, zmp_R);

  //////////////////////////////////////////////////////////////////////
  // fill up buffers with zmp reference and foot info

  // some space to hold zmp reference, foot trajectory, and stance info
  Eigen::ArrayXd zmpref(total_ticks);
  Eigen::ArrayXd footz(total_ticks);
  std::vector<stance_t> stance(total_ticks);

  size_t cur_tick = 0;

  double p = 0;
  double p_next = sway;
  stance_t s = DOUBLE_LEFT;


  for (size_t i=0; i<startup_ticks; ++i) {
    stance[cur_tick] = s;
    footz(cur_tick) = 0;
    zmpref(cur_tick++) = p;
  }

  for (size_t k=0; k<n_steps; ++k) {
    p = p_next;
    p_next = -p_next;
    for (size_t i=0; i<double_support_ticks; ++i) {
      stance[cur_tick] = s;
      footz(cur_tick) = 0;
      zmpref(cur_tick++) = p;
    }
    s = next_stance_table[s];
    for (size_t i=0; i<single_support_ticks; ++i) {
      double u = double(i)/double(single_support_ticks-1);
      double a = u*2*M_PI;
      stance[cur_tick] = s;
      footz(cur_tick) = 0.5*fz*(1 - cos(a)); // the Y component of a cycloid
      zmpref(cur_tick++) = p;
    }
    s = next_stance_table[s];
  }

  p = 0;
  for (size_t i=0; i<shutdown_ticks; ++i) {
    stance[cur_tick] = s;
    footz(cur_tick) = 0;
    zmpref(cur_tick++) = p;
  }

  assert(cur_tick == total_ticks);

  //////////////////////////////////////////////////////////////////////
  // We are now running our ZMP preview controller to generate a COM 
  // trajectory

  Eigen::Vector3d Y(0.0, 0.0, 0.0);
  double e = 0;
  p = 0;

  Eigen::MatrixXd com(total_ticks,3);
  Eigen::ArrayXd zmp(total_ticks);

  for (size_t i=0; i<total_ticks; ++i) {
    com.row(i) = Y.transpose();
    zmp(i) = p;
    p = preview.update(Y, e, zmpref.block(i, 0, total_ticks-i, 1));
  }

  //////////////////////////////////////////////////////////////////////
  // fill up a full body trajectory using COM & footstep info
  // generated above.
  //
  // NOTE: I am assuming a fixed world frame independent of the robot
  // in this section of the code, which is kind of a big nono

  Transform3Array xforms;
  const KinBody& kbody = hplus.kbody;
  HuboPlus::KState state;
  
  state.body_pos = vec3(0, 0, 0.85);
  state.body_rot = quat();
  
  state.jvalues.resize(kbody.joints.size(), 0.0);
  
  real deg = M_PI/180;
  const JointLookup& jl = hplus.jl;
  state.jvalues[jl("LSR")] =  15*deg;
  state.jvalues[jl("RSR")] = -15*deg;
  state.jvalues[jl("LSP")] =  20*deg;
  state.jvalues[jl("RSP")] =  20*deg;
  state.jvalues[jl("LEP")] = -40*deg;
  state.jvalues[jl("REP")] = -40*deg;

  kbody.transforms(state.jvalues, xforms);

  Transform3 desired[4];
  vec3 desiredCom;
  
  desired[0].setTranslation(vec3(0, fy, 0));
  desired[1].setTranslation(vec3(0, -fy, 0));

  HuboPlus::IKMode mode[4] = { 
    HuboPlus::IK_MODE_FIXED,
    HuboPlus::IK_MODE_FIXED,
    HuboPlus::IK_MODE_FIXED,
    HuboPlus::IK_MODE_FIXED,
  };

  TrajVector traj;
  
  // loop thru trajectory and make full-body joint trajectory
  for (size_t i=0; i<total_ticks; ++i) {

    int stance_foot = stance_foot_table[stance[i]];
    int swing_foot = swing_foot_table[stance[i]];

    if (swing_foot < 0) {

      mode[0] = HuboPlus::IK_MODE_SUPPORT;
      mode[1] = HuboPlus::IK_MODE_SUPPORT;
      desired[0].setTranslation(vec3(0,  fy, 0));
      desired[1].setTranslation(vec3(0, -fy, 0));

    } else {


      mode[swing_foot] = HuboPlus::IK_MODE_WORLD;
      mode[stance_foot] = HuboPlus::IK_MODE_SUPPORT;
      
      const double sy[2] = { 1, -1 };

      desired[stance_foot].setTranslation(vec3(0, sy[stance_foot]*fy, 0));
      
      double z = footz[i];
      desired[swing_foot].setTranslation(vec3(0, sy[swing_foot]*fy, z));

    }

    desiredCom = vec3(0, com(i), com_height);

    bool ikvalid[4];
    
    bool ok = hplus.comIK( state, desiredCom, desired, mode, 
			   HuboPlus::noGlobalIK(), xforms, 
			   0, 0, ikvalid );

    //for (int i=0; i<4; ++i) { std::cerr << ", " << ikvalid[i] << " "; }
    assert( ok && "comIK sadness :(" );

    zmp_traj_element_t cur;
    memset(&cur, 0, sizeof(cur));

    for (size_t hi=0; hi<hplus.huboJointOrder.size(); ++hi) {
      size_t ji = hplus.huboJointOrder[hi];
      if (ji != size_t(-1)) {
	cur.angles[hi] = state.jvalues[ji];
      }
      cur.stance = stance[i];
      for (int a=0; a<3; ++a) {
	double foffset = (a == 0) ? desired[stance_foot].translation().y() : 0;
	cur.com[1][a] = com(i,a) - foffset;
      }
      cur.com[2][0] = com_height;
      
    }

    traj.push_back(cur);

  }

  //////////////////////////////////////////////////////////////////////
  // ach_put goes after this line

  zmp_traj_t trajectory;

  int N;
  if(traj.size() > MAX_TRAJ_SIZE)
  {
    N = MAX_TRAJ_SIZE;
    fprintf(stdout, "Trajectory size has exceeded the maximum! (%d,%d)\n", int(traj.size()), MAX_TRAJ_SIZE);
  }
  else
    N = traj.size();

  trajectory.count = N;
  for( int t=0; t<N; t++ )
    memcpy( &(trajectory.traj[t]), &(traj[t]), sizeof(zmp_traj_element_t) );


  ach_put( &zmp_chan, &trajectory, sizeof(trajectory) );
  fprintf(stdout, "Message put\n");
/*
    for(int i=0; i<traj.size(); i++)
        for(int j=0; j<HUBO_JOINT_COUNT; j++)
            if(traj[i].angles[j] != 0)
                fprintf(stdout, "(%d,%d) %f\n", i, j, traj[i].angles[j]);
*/
/*
    int joint = RHR;
    for(int i=0; i<trajectory.count; i++)
    {
        fprintf(stdout, "%d: Angle %f:%f\n", i, trajectory.traj[i].angles[joint], traj[i].angles[joint] );
    }
*/
/*
  if (show_gui) {

    ZmpDemo demo(argc, argv, hplus, traj);

    demo.run();

  }
*/

  return 0;

}

