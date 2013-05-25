#ifndef ZMPDAEMON_H
#define ZMPDAEMON_H

#include "hubo-zmp.h"

#define CHAN_ZMP_CMD_NAME "hubo-zmp-cmd"

enum ik_error_sensitivity {
  ik_strict, // default
  ik_swing_permissive, // allows ik errors on swing foot when above 0.5 * step_height
  ik_sloppy // never ever ever ever run this on the robot
};


enum walktype {
  walk_canned,
  walk_line,
  walk_circle,
  walk_sidestep
};

typedef struct zmp_cmd
{
    walkState_t cmd_state;

    bool walk_continuous;
    double walk_dist;
    double walk_circle_radius;

    walktype walk_type;
    size_t max_step_count;
    double step_length;

    double footstep_y;
    double foot_liftoff_z;

    double sidewalk_dist;
    double sidestep_length;

    double com_height;
    double com_ik_ascl;

    double zmpoff_y;
    double zmpoff_x;

    double lookahead_time;

    double startup_time;
    double shutdown_time;
    double double_support_time;
    double single_support_time;

    double zmp_jerk_penalty;
    // size_t curTrajNumber; // Is this useful?

    ik_error_sensitivity ik_sense;

} zmp_cmd_t;


typedef enum {

    ZMP_STOPPED=0,
    ZMP_RUNNING,
    ZMP_IK_FAIL

} zmp_result_t;


typedef struct zmp_state
{
    zmp_result_t result;
} zmp_state_t;




#endif // ZMPDAEMON_H
