#include "hubo-zmp.h"
#include <HuboPlus.h>
#include <ZmpPreview.h>
#include <math.h>
#include <mzcommon/MzGlutApp.h>
#include <mzcommon/TimeUtil.h>
#include <getopt.h>

#include "zmpwalkgenerator.h"

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
      actualComVel[a] = cur.com[a][1];
      actualComAcc[a] = cur.com[a][2];
      for (int f=0; f<2; ++f) {
	forces[f][a] = cur.forces[f][a];
	torques[f][a] = cur.torque[f][a]; // TODO: FIXME: pluralization WTF?
      }
    }

    std::cerr << "current index: " << cur_index << "/" << traj.size() << ", stance=" << cur.stance << "\n";  
    
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

/*
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
  
/*
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


void usage(std::ostream& ostr) {
  ostr << 
    "usage: zmpdemo [OPTIONS] HUBOFILE.xml\n"
    "\n"
    "OPTIONS:\n"
    "\n"
    "  -g, --show-gui                    Show a GUI after computing trajectories.\n"
    "  -A, --use-ach                     Send trajectory via ACH after computing.\n"
    "  -f, --foot-y=NUMBER               Half-distance between feet\n"
    "  -L, --foot-liftoff=NUMBER         Vertical liftoff distance of swing foot\n"
    "  -x, --step-distance=NUMBER        Forward distance between front & rear foot\n"
    "  -Y, --zmp-y=NUMBER                Lateral distance from ankle to ZMP\n"
    "  -X, --zmp-x=NUMBER                Forward distance from ankle to ZMP\n"
    "  -h, --com-height=NUMBER           Height of the center of mass\n"
    "  -l, --lookahead-time=NUMBER       Lookahead window for ZMP preview controller\n"
    "  -p, --startup-time=NUMBER         Initial time spent with ZMP stationary\n"
    "  -n, --shutdown-time=NUMBER        Final time spent with ZMP stationary\n"
    "  -d, --double-support-time=NUMBER  Double support time\n"
    "  -s, --single-support-time=NUMBER  Single support time\n"
    "  -a, --angle-weight=NUMBER         Angle weight for COM IK\n"
    "  -c, --step-count=NUMBER           Number of steps to take\n"
    "  -R, --zmp-jerk-penalty=NUMBER     R-value for ZMP preview controller\n"
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


  bool show_gui = false;
  bool use_ach = false;

  double fy = 0.085; // half of horizontal separation distance between feet
  double zmpy = 0; // lateral displacement between zmp and ankle
  double zmpx = 0;
  double fz = 0.05; // foot liftoff height

  double circle_max_step_length = 0.2; // maximum distance between steps
  double circle_max_step_angle = M_PI / 12.0; // maximum angle between steps
  double circle_distance = 5.0; // distance to go along circle
  double circle_radius = 2.0; // radius of circle to move in

  double lookahead_time = 2.5;
  double startup_time = 1.0;
  double shutdown_time = 1.0;
  double double_support_time = 0.05;
  double single_support_time = 0.70;

  double com_height = 0.48; // height of COM above ANKLE
  double com_ik_ascl = 0;

  double zmp_R = 1e-8; // jerk penalty on ZMP controller

  const struct option long_options[] = {
    { "show-gui",            no_argument,       0, 'g' },
    { "use-ach",             no_argument,       0, 'A' },
    { "foot-y",              required_argument, 0, 'f' },
    { "foot-liftoff",        required_argument, 0, 'L' },
    { "step-distance",       required_argument, 0, 'x' },
    { "zmp-y",               required_argument, 0, 'Y' },
    { "zmp-x",               required_argument, 0, 'X' },
    { "com-height",          required_argument, 0, 'h' },
    { "lookahead-time",      required_argument, 0, 'l' },
    { "startup-time",        required_argument, 0, 'p' },
    { "shutdown-time",       required_argument, 0, 'n' },
    { "double-support-time", required_argument, 0, 'd' },
    { "single-support-time", required_argument, 0, 's' },
    { "step-count",          required_argument, 0, 'c' },
    { "angle-weight",        required_argument, 0, 'a' },
    { "zmp-jerk-penalty",    required_argument, 0, 'R' },
    { "help",                no_argument,       0, 'H' },
    { 0,                     0,                 0,  0  },
  };

  const char* short_options = "gAf:L:x:Y:X:h:l:p:n:d:s:c:a:R:H";

  int opt, option_index;

  while ( (opt = getopt_long(argc, argv, short_options, long_options, &option_index)) != -1 ) {
    switch (opt) {
    case 'g': show_gui = true; break;
    case 'A': use_ach = true; break;
    case 'f': fy = getdouble(optarg); break;
    case 'L': fz = getdouble(optarg); break;
    // case 'x': fy = getdouble(optarg); break; // FIXME: rebuild the options to support circle trajectory args
    case 'Y': zmpy = getdouble(optarg); break;
    case 'X': zmpx = getdouble(optarg); break;
    case 'h': com_height = getdouble(optarg); break;
    case 'l': lookahead_time = getdouble(optarg); break;
    case 'p': startup_time = getdouble(optarg); break;
    case 'n': shutdown_time = getdouble(optarg); break;
    case 'd': double_support_time = getdouble(optarg); break;
    case 's': single_support_time = getdouble(optarg); break;
    case 'a': com_ik_ascl = getdouble(optarg); break;
    // case 'c': n_steps = getlong(optarg); break;  // FIXME: rebuild the options to support circle trajectory args
    case 'R': zmp_R = getdouble(optarg); break;
    case 'H': usage(std::cout); exit(0); break;
    default:  usage(std::cerr); exit(1); break;
    }
  }


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
  if (!hubofile) { usage(std::cerr); exit(1); }
  HuboPlus hplus(hubofile);


  
  // const double& l6 = hplus.footAnkleDist;

  // double zmp_dt = 1.0/TRAJ_FREQ_HZ; // delta t for ZMP preview controller
  // size_t num_lookahead = seconds_to_ticks(lookahead_time); // lookahead window size
  // ZmpPreview preview(zmp_dt, com_height, num_lookahead, zmp_R);


  //////////////////////////////////////////////////////////////////////
  // build initial state

  // the actual state
  ZMPWalkGenerator walker;
  ZMPReferenceContext initContext;

  // helper variables and classes
  Transform3Array xforms;
  const KinBody& kbody = hplus.kbody;
  const JointLookup& jl = hplus.jl;
  double deg = M_PI/180; // for converting from degrees to radians

  // fill in the kstate
  initContext.state.body_pos = vec3(0, 0, 0.85);
  initContext.state.body_rot = quat();
  initContext.state.jvalues.resize(kbody.joints.size(), 0.0);
  initContext.state.jvalues[jl("LSR")] =  15*deg;
  initContext.state.jvalues[jl("RSR")] = -15*deg;
  initContext.state.jvalues[jl("LSP")] =  20*deg;
  initContext.state.jvalues[jl("RSP")] =  20*deg;
  initContext.state.jvalues[jl("LEP")] = -40*deg;
  initContext.state.jvalues[jl("REP")] = -40*deg;
  kbody.transforms(initContext.state.jvalues, xforms);
  
  // build and fill in the initial foot positions
  initContext.feet[0] = Transform3(quat(), vec3(0, fy, 0));
  initContext.feet[1] = Transform3(quat(), vec3(0, -fy, 0));

  // fill in the rest
  vec3 betweenFeet = (initContext.feet[0] * vec3() + initContext.feet[1] * vec3()) / 2.0;
  initContext.stance = DOUBLE_LEFT;
  initContext.comX = Eigen::Vector3d(betweenFeet.x(), 0.0, 0.0);
  initContext.comY = Eigen::Vector3d(betweenFeet.y(), 0.0, 0.0);
  initContext.eX = 0.0;
  initContext.eY = 0.0;
  initContext.pX = betweenFeet.x();
  initContext.pY = betweenFeet.y();
  
  walker.initialize(initContext);

  
  //////////////////////////////////////////////////////////////////////
  // build ourselves some footprints
  
  Transform3 initLeftTransform = initContext.feet[0];
  Transform3 initRightTransform = initContext.feet[1];
  vec3 initLeftPosition = initLeftTransform * vec3();
  vec3 initRightPosition = initRightTransform * vec3();
  
  Footprint initLeftFoot = Footprint(initLeftPosition.x(),
                                     initLeftPosition.y(),
                                     0.0, // FIXME: actually pull rotation out of initContext
                                     true);
  Footprint initRightFoot = Footprint(initRightPosition.x(),
                                      initRightPosition.y(),
                                      0.0, // FIXME: actually pull rotation out of initContext
                                      false);
  
  std::vector<Footprint> footprints = walkCircle(circle_radius,
                                                 circle_distance,
                                                 fy,
                                                 circle_max_step_length,
                                                 circle_max_step_angle,
                                                 &initLeftFoot,
                                                 &initRightFoot,
                                                 DOUBLE_LEFT);

  

  //////////////////////////////////////////////////////////////////////
  // and then build up the walker

  walker.stayDogStay(startup_time * TRAJ_FREQ_HZ);
  for(std::vector<Footprint>::iterator it = footprints.begin(); it != footprints.end(); it++) {
    walker.addFootstep(*it);
  }
  walker.stayDogStay(shutdown_time * TRAJ_FREQ_HZ);

  //////////////////////////////////////////////////////////////////////
  // have the walker run preview control and pass on the output

  walker.bakeIt();
  // validateOutputData(traj);

#ifdef HAVE_HUBO_ACH

  if (use_ach) {
    ach_channel_t zmp_chan;
    ach_open( &zmp_chan, HUBO_CHAN_ZMP_TRAJ_NAME, NULL );

    zmp_traj_t trajectory;
    memset( &trajectory, 0, sizeof(trajectory) );

    int N;
    if( (int)walker.traj.size() > MAX_TRAJ_SIZE )
      N = MAX_TRAJ_SIZE;
    else
      N = (int)walker.traj.size();

    trajectory.count = N;
    for(int i=0; i<N; i++)
      memcpy( &(trajectory.traj[i]), &(walker.traj[i]), sizeof(zmp_traj_element_t) );

    ach_put( &zmp_chan, &trajectory, sizeof(trajectory) );
    fprintf(stdout, "Message put\n");
  }

#endif

  if (show_gui) {

    ZmpDemo demo(argc, argv, hplus, walker.traj);

    demo.run();

  }


  return 0;
}


// Local Variables:
// c-basic-offset: 2
// End:
