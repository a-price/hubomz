#include "hubo-zmp.h"
#include <HuboPlus.h>
#include <ZmpPreview.h>
#include <math.h>
#include <mzcommon/MzGlutApp.h>
#include <mzcommon/TimeUtil.h>
#include <getopt.h>

using namespace fakerave;

typedef std::vector< zmp_traj_element_t > TrajVector;

size_t seconds_to_ticks(double s) {
  return size_t(round(s*TRAJ_FREQ_HZ));
}

double sigmoid(double x) {
  return 3*x*x - 2*x*x*x;
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
      animating = false;
      deltaCurrent(-5);
      glutPostRedisplay();
      break;
    case '+':
    case '=':
      animating = false;
      deltaCurrent(5);
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


void usage(std::ostream& ostr) {
  ostr << 
    "usage: zmpdemo [OPTIONS] HUBOFILE.xml\n"
    "\n"
    "OPTIONS:\n"
    "\n"
    "  -g, --show-gui                    Show a GUI after computing trajectories.\n"
    "  -h, --com-height=NUMBER           Height of the center of mass\n"
    "  -f, --foot-y=NUMBER               Half-distance between feet\n"
    "  -L, --foot-liftoff=NUMBER         Vertical liftoff distance of swing foot\n"
    "  -z, --zmp-y=NUMBER                Lateral distance from ankle to ZMP\n"
    "  -l, --lookahead-time=NUMBER       Lookahead window for ZMP preview controller\n"
    "  -p, --startup-time=NUMBER         Initial time spent with ZMP stationary\n"
    "  -n, --shutdown-time=NUMBER        Final time spent with ZMP stationary\n"
    "  -d, --double-support-time=NUMBER  Double support time\n"
    "  -s, --single-support-time=NUMBER  Single support time\n"
    "  -a, --angle-weight=NUMBER         Angle weight for COM IK\n"
    "  -H, --help                        See this message\n";
    
}

double getdouble(const char* str) {
  char* endptr;
  double d = strtod(str, &endptr);
  if (!endptr || *endptr) {
    std::cerr << "Error parsing number on command line!\n\n";
    usage(std::cerr);
    exit(1);
  }
  return d;
}

long getlong(const char* str) {
  char* endptr;
  long d = strtol(str, &endptr, 10);
  if (!endptr || *endptr) {
    std::cerr << "Error parsing number on command line!\n\n";
    usage(std::cerr);
    exit(1);
  }
  return d;
}



int main(int argc, char** argv) {

  if (argc < 2) {
    usage(std::cerr);
    return 1;
  }

  ach_channel_t zmp_chan;
  ach_open( &zmp_chan, HUBO_CHAN_ZMP_TRAJ_NAME, NULL );
  

  HuboPlus hplus(argv[1]);

  bool show_gui = false;

  double fy = 0.085; // half of horizontal separation distance between feet
  double zmpy = 0; // lateral displacement between zmp and ankle
  double fz = 0.05; // foot liftoff height

  double lookahead_time = 2.5;
  double startup_time = 1.0;
  double shutdown_time = 1.0;
  double double_support_time = 0.05;
  double single_support_time = 0.70;
  double com_height = 0.58; // height of COM above ground
  double com_ik_ascl = 0;

  size_t n_steps = 8; // how many steps?

  const struct option long_options[] = {
    { "show-gui",            no_argument,       0, 'g' },
    { "foot-y",              required_argument, 0, 'f' },
    { "foot-liftoff",        required_argument, 0, 'L' },
    { "zmp-y",               required_argument, 0, 'z' },
    { "com-height",          required_argument, 0, 'h' },
    { "lookahead-time",      required_argument, 0, 'l' },
    { "startup-time",        required_argument, 0, 'p' },
    { "shutdown-time",       required_argument, 0, 'n' },
    { "double-support-time", required_argument, 0, 'd' },
    { "single-support-time", required_argument, 0, 's' },
    { "step-count",          required_argument, 0, 'c' },
    { "angle-weight",        required_argument, 0, 'a' },
    { "help",                no_argument,       0, 'H' },
    { 0,                     0,                 0,  0  },
  };

  const char* short_options = "gf:L:z:h:l:p:n:d:s:c:a:H";

  int opt, option_index;

  while ( (opt = getopt_long(argc, argv, short_options, long_options, &option_index)) != -1 ) {
    switch (opt) {
    case 'g': show_gui = true; break;
    case 'f': fy = getdouble(optarg); break;
    case 'L': fz = getdouble(optarg); break;
    case 'z': zmpy = getdouble(optarg); break;
    case 'h': com_height = getdouble(optarg); break;
    case 'l': lookahead_time = getdouble(optarg); break;
    case 'p': startup_time = getdouble(optarg); break;
    case 'n': shutdown_time = getdouble(optarg); break;
    case 'd': double_support_time = getdouble(optarg); break;
    case 's': single_support_time = getdouble(optarg); break;
    case 'a': com_ik_ascl = getdouble(optarg); break;
    case 'c': n_steps = getlong(optarg); break;
    case 'H': usage(std::cout); exit(0); break;
    default:  usage(std::cerr); exit(1); break;
    }
  }

  size_t num_lookahead = seconds_to_ticks(lookahead_time); // lookahead window size
  size_t startup_ticks = seconds_to_ticks(startup_time); // hold ZMP at center for 1s at start
  size_t shutdown_ticks = seconds_to_ticks(shutdown_time); // hold ZMP at center for 1s at end
  size_t double_support_ticks = seconds_to_ticks(double_support_time); // .05s of double support each step
  size_t single_support_ticks = seconds_to_ticks(single_support_time); // .70s of single support each step

  const char* hubofile = 0;

  while (optind < argc) {
    if (!hubofile) {
      hubofile = argv[optind++];
    } else {
      std::cerr << "Error: extra arguments on command line.\n\n";
      usage(std::cerr);
      exit(1);
    }
  }

  HuboPlus hplus(hubofile);

  size_t step_ticks = double_support_ticks + single_support_ticks;
  size_t total_ticks = startup_ticks + n_steps * step_ticks + shutdown_ticks;

  double zmp_R = 1e-10; // gain for ZMP controller
  double zmp_dt = 1.0/TRAJ_FREQ_HZ; // delta t for ZMP preview controller
  
  TimeStamp t0 = TimeStamp::now();

  ZmpPreview preview(zmp_dt, com_height, num_lookahead, zmp_R);

  //////////////////////////////////////////////////////////////////////
  // fill up buffers with zmp reference and foot info

  // some space to hold zmp reference, foot trajectory, and stance info
  Eigen::ArrayXd zmpref(total_ticks);
  Eigen::ArrayXd footz(total_ticks);
  std::vector<stance_t> stance(total_ticks);

  size_t cur_tick = 0;

  double p = 0;
  double p_next = fy-zmpy;
  stance_t s = DOUBLE_LEFT;


  for (size_t i=0; i<startup_ticks; ++i) {
    stance[cur_tick] = s;
    footz(cur_tick) = 0;
    zmpref(cur_tick++) = p;
  }

  for (size_t k=0; k<n_steps; ++k) {
    for (size_t i=0; i<double_support_ticks; ++i) {
      double u = double(i)/double(double_support_ticks-1);
      stance[cur_tick] = s;
      footz(cur_tick) = 0;
      zmpref(cur_tick++) = p + sigmoid(u)*(p_next-p);
    }
    p = p_next;
    p_next = -p_next;
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

  TimeStamp t1 = TimeStamp::now();

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

  TimeStamp t2 = TimeStamp::now();

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
			   com_ik_ascl, 0, ikvalid );

    if (!ok) {
      kbody.transforms(state.jvalues, xforms);
      std::cerr << "IK FAILURE!\n\n";
      std::cerr << "  body: " << state.xform() << "\n\n";
      for (int i=0; i<4; ++i) { 
	if (mode[i] != HuboPlus::IK_MODE_FIXED &&
	    mode[i] != HuboPlus::IK_MODE_FREE) {

	  Transform3 fk = kbody.manipulatorFK(xforms, i);
	  if (mode[i] == HuboPlus::IK_MODE_WORLD || 
	      mode[i] == HuboPlus::IK_MODE_SUPPORT) {
	    fk = state.xform() * fk;
	  }
	  vec3 dp, dq;
	  deltaTransform(desired[i], fk, dp, dq);
	  std::cerr << "  " << kbody.manipulators[i].name << ":\n";
	  std::cerr << "    valid:   " << ikvalid[i] << "\n";
	  std::cerr << "    desired: " << desired[i] << "\n";
	  std::cerr << "    actual:  " << fk << "\n";
	  std::cerr << "    dp:      " << dp << " with norm " << dp.norm() << "\n";
	  std::cerr << "    dq:      " << dq << " with norm " << dq.norm() << "\n";
	  std::cerr << "\n";
	}
      }
      exit(1);
    }

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

  TimeStamp t3 = TimeStamp::now();

  double sim_time = total_ticks * zmp_dt;
  double d0 = (t1-t0).toDouble();
  double d1 = (t2-t1).toDouble();
  double d2 = (t3-t2).toDouble();
  double total_time = (t3-t0).toDouble();

  std::cerr << "Generated reference trajectories in:      " << d0 << "s\n";
  std::cerr << "Generated center of mass trajectories in: " << d1 << "s\n";
  std::cerr << "Generated full-body trajectories in:      " << d2 << "s\n";
  std::cerr << "Total time:                               " << total_time << "s\n";
  std::cerr << "Execution time:                           " << sim_time << "s\n";
  std::cerr << "Speedup over real-time:                   " << sim_time/total_time << "\n";


  ach_put( &zmp_chan, &trajectory, sizeof(trajectory) );
  fprintf(stdout, "Message put\n");

  if (show_gui) {

    ZmpDemo demo(argc, argv, hplus, traj);

    demo.run();

  }


  return 0;

}

