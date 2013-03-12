#ifndef _HUBOACHDRIVER_H_
#define _HUBOACHDRIVER_H_

#include <hubo.h>
#include <ach.h>

class HuboAchDriver {
public:
  
  // TODO: deal
  HuboAchDriver();
  ~HuboAchDriver();

  static bool useable(ach_status_t r);

  ach_status_t get(hubo_ref_t* ref);
  ach_status_t put(hubo_ref_t* ref);

  ach_status_t get(hubo_state_t* state);
  ach_status_t put(hubo_state_t* state);

  ach_channel_t chan_hubo_ref;
  ach_channel_t chan_hubo_state;

};

#endif
