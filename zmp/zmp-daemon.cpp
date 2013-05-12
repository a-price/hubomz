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
#include <iostream>

// for keyboard interrupt
static int tty_unbuffered(int);     
static void tty_atexit(void);
static int tty_reset(int);
static void keyboard_init();
static struct termios save_termios;
static int  ttysavefd = -1;

using namespace fakerave;

typedef std::vector< zmp_traj_element_t > TrajVector;

size_t seconds_to_ticks(double s) {
  return size_t(round(s*TRAJ_FREQ_HZ));
}

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
          std::cerr << "change in joint " << jointNames[j] << " is larger than " << jointVelTol << "(" << jointVel << ")\n";
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
    "  -A, --use-ach                     Send trajectory via ACH after computing.\n"
    "  -I, --ik-errors                   IK error handling: strict/permissive/sloppy\n"
    "  -w, --walk-type                   Set type: canned/line/circle\n"
    "  -D, --walk-distance               Set maximum distance to walk\n"
    "  -r, --walk-circle-radius          Set radius for circle walking\n"
    "  -c, --max-step-count=NUMBER       Set maximum number of steps\n"
    "  -y, --foot-separation-y=NUMBER    Half-distance between feet\n"
    "  -z, --foot-liftoff-z=NUMBER       Vertical liftoff distance of swing foot\n"
    "  -l, --step-length=NUMBER          Max length of footstep\n"
    "  -S, --walk-sideways               Should we walk sideways? (canned gait only)\n"
    "  -h, --com-height=NUMBER           Height of the center of mass\n"
    "  -a, --comik-angle-weight=NUMBER   Angle weight for COM IK\n"
    "  -Y, --zmp-offset-y=NUMBER         Lateral distance from ankle to ZMP\n"
    "  -X, --zmp-offset-x=NUMBER         Forward distance from ankle to ZMP\n"
    "  -T, --lookahead-time=NUMBER       Lookahead window for ZMP preview controller\n"
    "  -p, --startup-time=NUMBER         Initial time spent with ZMP stationary\n"
    "  -n, --shutdown-time=NUMBER        Final time spent with ZMP stationary\n"
    "  -d, --double-support-time=NUMBER  Double support time\n"
    "  -s, --single-support-time=NUMBER  Single support time\n"
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

enum walktype {
  walk_canned,
  walk_line,
  walk_circle
};

walktype getwalktype(const std::string& s) {
  if (s == "canned") {
    return walk_canned;
  } else if (s == "line") {
    return walk_line;
  } else if (s == "circle") {
    return walk_circle;
  } else {
    std::cerr << "bad walk type " << s << "\n";
    usage(std::cerr);
    exit(1);
  }
}

ZMPWalkGenerator::ik_error_sensitivity getiksense(const std::string& s) {
  if (s == "strict") {
    return ZMPWalkGenerator::ik_strict;
  } else if (s == "sloppy") {
    return ZMPWalkGenerator::ik_sloppy;
  } else if (s == "permissive") {
    return ZMPWalkGenerator::ik_swing_permissive;
  } else {
    std::cerr << "bad ik error sensitivity " << s << "\n";
    usage(std::cerr);
    exit(1);
  }
}

int main(int argc, char** argv)
{
  if (argc < 2) {
    usage(std::cerr);
    return 1;
  }
  FILE *jointsFile;
  FILE *comFile;
  jointsFile = fopen("joints.data","w");
  comFile = fopen("com.data","w");
  bool use_ach = false;

  walktype walk_type = walk_line;
  double walk_circle_radius = 5.0;
  double walk_dist = .3;
  size_t max_step_count = 20;
  double step_length = 0.1;

  double footsep_y = 0.0885; // half of horizontal separation distance between feet
  double foot_liftoff_z = 0.04; // foot liftoff height

  double sidestep_length = 0.01;
  bool walk_sideways = false;

  double com_height = 0.5; // height of COM above ANKLE
  double com_ik_ascl = 0;

  double zmpoff_y = 0; // lateral displacement between zmp and ankle
  double zmpoff_x = 0.038;

  double lookahead_time = 2.5;

  double startup_time = 1.0;
  double shutdown_time = 1.0;
  double double_support_time = 0.01;
  double single_support_time = 0.50;

  double zmp_jerk_penalty = 1e-8; // jerk penalty on ZMP controller
  size_t curTrajNumber = 0; // current trajectory number

  ZMPWalkGenerator::ik_error_sensitivity ik_sense = ZMPWalkGenerator::ik_swing_permissive;

  const struct option long_options[] = {
    { "use-ach",             no_argument,       0, 'A' },
    { "ik-errors",           required_argument, 0, 'I' },
    { "walk-type",           required_argument, 0, 'w' },
    { "walk-distance",       required_argument, 0, 'D' },
    { "walk-circle-radius",  required_argument, 0, 'r' },
    { "max-step-count",      required_argument, 0, 'c' },
    { "foot-separation-y",   required_argument, 0, 'y' },
    { "foot-liftoff-z",      required_argument, 0, 'z' },
    { "step-length",         required_argument, 0, 'l' },
    { "walk-sideways",       no_argument,       0, 'S' },
    { "com-height",          required_argument, 0, 'h' },
    { "comik-angle-weight",  required_argument, 0, 'a' },
    { "zmp-offset-y",        required_argument, 0, 'Y' },
    { "zmp-offset-x",        required_argument, 0, 'X' },
    { "lookahead-time",      required_argument, 0, 'T' },
    { "startup-time",        required_argument, 0, 'p' },
    { "shutdown-time",       required_argument, 0, 'n' },
    { "double-support-time", required_argument, 0, 'd' },
    { "single-support-time", required_argument, 0, 's' },
    { "zmp-jerk-penalty",    required_argument, 0, 'R' },
    { "help",                no_argument,       0, 'H' },
    { 0,                     0,                 0,  0  },
  };

  const char* short_options = "AI:w:D:r:c:y:z:l:Sh:a:Y:X:T:p:n:d:s:R:H";

  int opt, option_index;


  while ( (opt = getopt_long(argc, argv, short_options, long_options, &option_index)) != -1 ) {
    switch (opt) {
    case 'A': use_ach = true; break;
    case 'I': ik_sense = getiksense(optarg); break;
    case 'w': walk_type = getwalktype(optarg); break;
    case 'D': walk_dist = getdouble(optarg); break;
    case 'r': walk_circle_radius = getdouble(optarg); break;
    case 'c': max_step_count = getlong(optarg); break;
    case 'y': footsep_y = getdouble(optarg); break;
    case 'z': foot_liftoff_z = getdouble(optarg); break;
    case 'l': step_length = getdouble(optarg); break;
    case 'S': walk_sideways = true; break;
    case 'h': com_height = getdouble(optarg); break;
    case 'a': com_ik_ascl = getdouble(optarg); break;
    case 'Y': zmpoff_y = getdouble(optarg); break;
    case 'X': zmpoff_x = getdouble(optarg); break;
    case 'T': lookahead_time = getdouble(optarg); break;
    case 'p': startup_time = getdouble(optarg); break;
    case 'n': shutdown_time = getdouble(optarg); break;
    case 'd': double_support_time = getdouble(optarg); break;
    case 's': single_support_time = getdouble(optarg); break;
    case 'R': zmp_jerk_penalty = getdouble(optarg); break;
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
  if (!hubofile) { 
    std::cerr << "Please supply a huboplus file!\n\n";
    usage(std::cerr); 
    exit(1); 
  }

  // load in openRave hubo model
  HuboPlus hplus(hubofile);

  // initialize keyboard interrupt
  keyboard_init();
  char key = '0';
  char prevKey = '1';

  ach_channel_t zmp_chan;

  if (use_ach)
  {
    std::cout << "opening ach channel\n";
    ach_status r = ach_open( &zmp_chan, HUBO_CHAN_ZMP_TRAJ_NAME, NULL );
    fprintf(stdout, "%s \n", ach_result_to_string(r));
  }

  // the actual state
  ZMPWalkGenerator walker(hplus,
                          ik_sense,
                          com_height,
                          zmp_jerk_penalty,
                          zmpoff_x,
                          zmpoff_y,
                          com_ik_ascl,
                          single_support_time,
                          double_support_time,
                          startup_time,
                          shutdown_time,
                          foot_liftoff_z,
                          lookahead_time);

  // BUILD INITIAL STATE

  // initial ZMPReferenceContext that's used for each consecutive trajectory
  ZMPReferenceContext initContext;

  // helper variables and classes
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
  
  // build and fill in the initial foot positions

  Transform3 starting_location(quat::fromAxisAngle(vec3(0,0,1), 0));
  initContext.feet[0] = Transform3(starting_location.rotation(), starting_location * vec3(0, footsep_y, 0));
  initContext.feet[1] = Transform3(starting_location.rotation(), starting_location * vec3(0, -footsep_y, 0));

  // fill in the rest
  initContext.stance = DOUBLE_LEFT;
  initContext.comX = Eigen::Vector3d(zmpoff_x, 0.0, 0.0);
  initContext.comY = Eigen::Vector3d(0.0, 0.0, 0.0);
  initContext.eX = 0.0;
  initContext.eY = 0.0;
  initContext.pX = 0.0;
  initContext.pY = 0.0;

  bool ready = false;
  bool printStopped = false;
  walkState_t walkState = STOP;
  walkTransition_t walkTransition = STAY_STILL;
  double startTick = 0;
  double curTrajStartTick = 0;
  zmp_traj_t trajectory;
  bool fileClosed = false;
  std::vector<ZMPReferenceContext> fullRefTraj;
  Footprint initFoot;

  // main loop
  while(true)
  {
    ready = false;

    // check for next input command
    while(ready == false)
    {
      // see if key has been pressed
      if( read(STDIN_FILENO, &key, 1) == 1 )
      {
        std::cout << "Reading keyboard input\n";
        // if user pressed a new key, then process walk command
        if( key != prevKey && (key == 'i' || key == 'j' || key == 'k' || key == 'm' || key == 'l'))
        {
          switch (key)
          {
              case 'i':
                  // walk forward
                  std::cout << "walking forwards\n";
                  walk_sideways = false;
                  step_length = abs(step_length);
                  prevKey = 'i';
                  ready = true;
                  walkState = WALKING_FORWARD;
                  walkTransition = SWITCH_WALK;
                  curTrajNumber = 0;
                  startTick = 0;
                  initContext.eX = 0;
                  initContext.eY = 0;
                  break;
              case 'm':
                  // walk backwards
                  std::cout << "walking backwards\n";
                  walk_sideways = false;
                  step_length = -abs(step_length);
                  prevKey = 'm';
                  ready = true;
                  walkState = WALKING_BACKWARD;
                  walkTransition = SWITCH_WALK;
                  curTrajNumber = 0;
                  startTick = 0;
                  initContext.eX = 0;
                  initContext.eY = 0;
                  break;
              case 'j':
                  // sidestep right
                  std::cout << "sidestepping right\n";
                  walk_sideways = true;
                  step_length = abs(sidestep_length);
                  prevKey = 'j';
                  ready = true;
                  walkState = SIDESTEPPING_LEFT;
                  walkTransition = SWITCH_WALK;
                  curTrajNumber = 0;
                  startTick = 0;
                  initContext.eX = 0;
                  initContext.eY = 0;
                  break;
              case 'l':
                  // sidestep left
                  std::cout << "sidestepping left\n";
                  walk_sideways = true;
                  step_length = -abs(sidestep_length);
                  prevKey = 'l';
                  ready = true;
                  walkState = SIDESTEPPING_RIGHT;
                  walkTransition = SWITCH_WALK;
                  curTrajNumber = 0;
                  startTick = 0;
                  initContext.eX = 0;
                  initContext.eY = 0;
                  break;
              case 'k':
                  // walk to standstill
                  std::cout << "walking to a stop\n";
                  prevKey = 'k';
                  ready = true;
                  walkState = STOP;
                  walkTransition = WALK_TO_STOP;
                  curTrajNumber = 0;
                  startTick = 0;
                  initContext.eX = 0;
                  initContext.eY = 0;
                  break;
              default:
                  break;
          }
        }
        // if no new key was pressed keep doing what we were doing before
        else
        {
          std::cout << "continue whatever we were doing before\n";
          key = prevKey;
          break;
        }
      }
      // else if a new key hasn't been pressed, but we've started walking
      else if(key == prevKey)
      {
        switch(walkState)
        {
          case WALKING_FORWARD:
            std::cout << "still walking forward\n";
            printStopped = true;
            walkTransition = KEEP_WALKING;
            break;
          case WALKING_BACKWARD:
            std::cout << "still walking forward\n";
            printStopped = true;
            walkTransition = KEEP_WALKING;
            break;
          case SIDESTEPPING_LEFT:
            std::cout << "still sidestepping left\n";
            printStopped = true;
            walkTransition = KEEP_WALKING;
            break;
          case SIDESTEPPING_RIGHT:
            std::cout << "still sidestepping right\n";
            printStopped = true;
            walkTransition = KEEP_WALKING;
            break;
          case STOP:
            if(printStopped == true)
            {
              printStopped = false;
              std::cout << "still stopped\n";
            }
            walkTransition = STAY_STILL;
            break;
        }
        break;
      }
    } // end of while loop




    if(walkState == STOP && walkTransition == WALK_TO_STOP)
    {
      trajectory.walkState = STOP;
      trajectory.walkTransition = WALK_TO_STOP;
      if (use_ach)
        ach_put( &zmp_chan, &trajectory, sizeof(trajectory) );
    }

    // ####################################
    // ####### GENERATE TRAJECTORY ########
    // ####################################
    // generate new trajectory if KEEP_WALKING or START_WALKING
    if( walkTransition == SWITCH_WALK || walkTransition == KEEP_WALKING )
    {
      curTrajNumber++;
      std::cout << "\n\nCURRENT TRAJECTORY #: " << curTrajNumber << "\n\n";

//      if( curTrajNumber != 1 && walkTransition == SWITCH_WALK )
//        initContext = walker.getNextInitContext();

      // apply COM IK for init context
      walker.applyComIK(initContext);

      /*
      walker.traj.resize(1);
      walker.refToTraj(initContext, walker.traj.back());
      */

      walker.initialize(initContext);
      
      //////////////////////////////////////////////////////////////////////
      // build ourselves some footprints

      if(initContext.stance == SINGLE_LEFT || initContext.stance == DOUBLE_LEFT)     
        initFoot = Footprint(initContext.feet[0], true);
      else
        initFoot = Footprint(initContext.feet[1], false);

      std::vector<Footprint> footprints;

      switch (walk_type)
      {
        case walk_circle:
        {
          double circle_max_step_angle = M_PI / 12.0; // maximum angle between steps TODO: FIXME: add to cmd line??
          footprints = walkCircle(walk_circle_radius,
                                  walk_dist,
                                  footsep_y,
                                  step_length,
                                  circle_max_step_angle,
                                  initFoot);
          break;
        }
        case walk_line:
        {
          footprints = walkLine(walk_dist,
                                footsep_y,
                                step_length,
                                initFoot);
          break;
        }
        default:
        {
          double cur_x[2] = { 0, 0 };
          double cur_y[2] = { 0, 0 };

          cur_y[0] =  footsep_y;
          cur_y[1] = -footsep_y;

          for (size_t i=0; i<max_step_count; ++i)
          {
            bool is_left = i%2;
            if (walk_sideways && step_length < 0) { is_left = !is_left; }
            int swing = is_left ? 0 : 1;
            int stance = 1-swing;
            if (walk_sideways)
            {
              cur_y[swing] -= step_length;
            } 
            else
            {
              // bring feet together
              if (i + 1 == max_step_count)
              {
                cur_x[swing] = cur_x[stance];
              }
              // take a full step
              else
              {
                cur_x[swing] = cur_x[stance] + 0.5*step_length;
              }
            }
            footprints.push_back(Footprint(cur_x[swing], cur_y[swing], 0, is_left));
          }

          break;
        }
      }

//      if (footprints.size() > max_step_count)
//      {
//        footprints.resize(max_step_count);
      //  if (footprints.size() > max_step_count + end_steps) {
      //    footprints.resize(max_step_count + end_steps);
//      }

      //////////////////////////////////////////////////////////////////////
      // and then build up the walker

      // if we are walking from a standstill, add startup ticks where the zmp is stationary
      if(walkTransition != KEEP_WALKING)
        walker.stayDogStay(startup_time * TRAJ_FREQ_HZ);

      curTrajStartTick = startTick;

      // add footsteps to ref, which is a ZMPReferenceContext,
      // which includes swingfoot trajectory, set next traj's initContext
      size_t footprintIndex=1;
      for(std::vector<Footprint>::iterator it = footprints.begin(); it != footprints.end(); it++) {
        if( footprintIndex == footprints.size() )
        {
          startTick = walker.getStartTick();
        }
        walker.addFootstep(*it);
        footprintIndex++;
      }

      // add shutdown ticks where the zmp is stationary
      walker.stayDogStay(shutdown_time * TRAJ_FREQ_HZ);
      

      //////////////////////////////////////////////////////////////////////
      // have the walker run preview control and pass on the output
      walker.bakeIt();
      initContext = walker.getNextInitContext((int)startTick);
//          std::cout << "\nleft foot: " << initContext.feet[0].translation()
//                    << "\nrigt foot: " << initContext.feet[1].translation()
//                    << "\neX, eY: " << initContext.eX << ", " << initContext.eY
//                    << "\ncomX: " << initContext.comX.transpose()
//                    << "\ncomY: " << initContext.comY.transpose()
//                    << std::endl;
//      fullRefTraj = walker.getRefTraj();
      walker.clearRef();
      validateOutputData(walker.traj);

      if (use_ach) {
        //############################
        //### SEND TRAJ TO WALKER ####
        //############################
        memset( &trajectory, 0, sizeof(trajectory) );

        // keep size of traj to MAX_TRAJ_SIZE
        int N;
        if( (int)walker.traj.size() > MAX_TRAJ_SIZE )
          N = MAX_TRAJ_SIZE;
        else
          N = (int)walker.traj.size();

        std::cout << "N: " << N << std::endl;

        // set variables for output to walker
        trajectory.count = N;//(uint16_t)N;
        trajectory.trajNumber = curTrajNumber;//(uint16_t)curTrajNumber;
        trajectory.walkState = walkState; 
        trajectory.walkTransition = walkTransition;
        trajectory.startTick = curTrajStartTick;//(uint16_t)curTrajStartTick;

        std::cout << "startTick for curTraj: " << curTrajStartTick << "\n";
        std::cout << "startTick for nextTraj: " << startTick << "\n";
        // put each 
        for(int i=0; i<N; i++)
        {
          memcpy( &(trajectory.traj[i]), &(walker.traj[i]), sizeof(zmp_traj_element_t) );
/*          if(i < startTick && curTrajNumber < 6 && fileClosed == false)
          //if(curTrajNumber == 1 && fileClosed == false)
          {
            //fprintf(jointsFile, "%llu, %llu, %llu, %llu, %llu, %llu\n", trajectory.traj[i].angles[RHY],
            fprintf(jointsFile, "%f, %f, %f, %f, %f, %f\n", trajectory.traj[i].angles[RHY],
                                                            trajectory.traj[i].angles[RHR],
                                                            trajectory.traj[i].angles[RHP],
                                                            trajectory.traj[i].angles[RKN],
                                                            trajectory.traj[i].angles[RAP],
                                                            trajectory.traj[i].angles[RAR]);
            fprintf(comFile, "%f, %f, %f, %f, %f, %f, %f, %f\n", fullRefTraj.at(i).comX(0),
                                                                     fullRefTraj.at(i).comX(1),
                                                                     fullRefTraj.at(i).comX(2),
                                                                     fullRefTraj.at(i).comY(0),
                                                                     fullRefTraj.at(i).comY(1),
                                                                     fullRefTraj.at(i).comY(2),
                                                                     fullRefTraj.at(i).feet[0].translation().z(),
                                                                     fullRefTraj.at(i).feet[1].translation().z());
*/          }
        }
        ach_status r = ach_put( &zmp_chan, &trajectory, sizeof(trajectory) );
        fprintf(stdout, "%s \n", ach_result_to_string(r));
        fprintf(stdout, "Message put\n");
      }
//      if(curTrajNumber == 6 && fileClosed == false)
      {
        fclose(jointsFile);
        fclose(comFile);
        fileClosed = true;
      }
    }
  }
  tty_reset(STDIN_FILENO);
  return 0;
}


static int
tty_unbuffered(int fd)      /* put terminal into a raw mode */
{
    struct termios  buf;

    if (tcgetattr(fd, &buf) < 0)
        return(-1);
        
    save_termios = buf; /* structure copy */

    /* echo off, canonical mode off */
    buf.c_lflag &= ~(ECHO | ICANON);

    /* 1 byte at a time, no timer */
    buf.c_cc[VMIN] = 1;
    buf.c_cc[VTIME] = 0;
    if (tcsetattr(fd, TCSAFLUSH, &buf) < 0)
        return(-1);

    ttysavefd = fd;
    return(0);
}

static int
tty_reset(int fd)       /* restore terminal's mode */
{
    if (tcsetattr(fd, TCSAFLUSH, &save_termios) < 0)
        return(-1);
    return(0);
}

static void
tty_atexit(void)        /* can be set up by atexit(tty_atexit) */
{
    if (ttysavefd >= 0)
        tty_reset(ttysavefd);
}

static void
keyboard_init()
{
   /* make stdin unbuffered */
    if (tty_unbuffered(STDIN_FILENO) < 0) {
        std::cout << "Set tty unbuffered error" << std::endl;
        exit(1);
    }

    atexit(tty_atexit);

    /* nonblock I/O */
    int flags;
    if ( (flags = fcntl(STDIN_FILENO, F_GETFL, 0)) == 1) {
        perror("fcntl get flag error");
        exit(1);
    }
    if (fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK) == -1) {
        perror("fcntl set flag error");
        exit(1);
    }
}

// Local Variables:
// c-basic-offset: 2
// End:
