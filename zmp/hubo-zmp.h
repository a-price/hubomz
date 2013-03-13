#ifndef _HUBO_ZMP_H_
#define _HUBO_ZMP_H_

#include <stdlib.h>

#ifdef HAVE_HUBO_ACH
#include <hubo.h>
#else
#include <hubo_joint_count.h>
#endif

enum stance_t {
  DOUBLE_LEFT  = 0,
  DOUBLE_RIGHT = 1,
  SINGLE_LEFT  = 2,
  SINGLE_RIGHT = 3,
};

typedef struct zmp_traj_element {
  double angles[HUBO_JOINT_COUNT];
  double com[3][3]; // XYZ pos/vel/accel in frame of stance foot
  stance_t stance;
} zmp_traj_element_t;

enum {
  TRAJ_FREQ_HZ = 200,
  MAX_TRAJ_SIZE = 1000,
};

typedef struct zmp_traj {
  zmp_traj_element_t traj[MAX_TRAJ_SIZE];
  size_t count;
} zmp_traj_t;

#define HUBO_CHAN_ZMP_TRAJ_NAME "hubo-zmp-traj"

#endif


