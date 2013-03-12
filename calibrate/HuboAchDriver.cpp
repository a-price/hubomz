#include "HuboAchDriver.h"


#include <stdlib.h>
#include <stdio.h>
#include <sched.h>
#include <sys/mman.h>
#include <assert.h>

// Priority
#define MY_PRIORITY (49)/* we use 49 as the PRREMPT_RT use 50
			    as the priority of kernel tasklets
			    and interrupt handler by default */

#define MAX_SAFE_STACK (1024*1024) /* The maximum stack size which is
				   guaranteed safe to access without
				   faulting */
// Priority
#define MY_PRIORITY (49)/* we use 49 as the PRREMPT_RT use 50
			    as the priority of kernel tasklets
			    and interrupt handler by default */

#define MAX_SAFE_STACK (1024*1024) /* The maximum stack size which is
				   guaranteed safe to access without
				   faulting */

int hubo_debug = 0;

void stack_prefault(void) {
  unsigned char dummy[MAX_SAFE_STACK];
  memset( dummy, 0, MAX_SAFE_STACK );
}

bool HuboAchDriver::useable(ach_status_t r) {
  return r == ACH_OK || r == ACH_MISSED_FRAME;
}

template <class Tdata>
static inline ach_status_t get(ach_channel_t* chan, 
			       Tdata* data,
			       const char* desc) {
  
  size_t fs;
  ach_status_t r = ach_get(chan, data, sizeof(Tdata), &fs, NULL, ACH_O_LAST);
  
  if (!HuboAchDriver::useable(r)) {
    if (hubo_debug) {
      fprintf(stderr, "get %s: %s\n", desc, ach_result_to_string(r));
    }
  } else { // is useable
    assert( fs == sizeof(Tdata) );
  }

  return r;
  
}

template <class Tdata>
static inline ach_status_t put(ach_channel_t* chan,
			       Tdata* data,
			       const char* desc) {

  ach_status_t r = ach_put(chan, data, sizeof(Tdata));

  if (r != ACH_OK) {
    if (hubo_debug) {
      fprintf(stderr, "put %s: %s\n", desc, ach_result_to_string(r));
    }
  }

  return r;

}

HuboAchDriver::HuboAchDriver() {

  

  ach_status_t r = ach_open(&chan_hubo_ref, HUBO_CHAN_REF_NAME , NULL);
  assert( "open ref" && ACH_OK == r );
  
  r = ach_open(&chan_hubo_state, HUBO_CHAN_STATE_NAME , NULL);
  assert( "open state" && ACH_OK == r );

}

HuboAchDriver::~HuboAchDriver() {
}



ach_status_t HuboAchDriver::get(hubo_ref_t* ref) {
  return ::get(&chan_hubo_ref, ref, "ref");
}

ach_status_t HuboAchDriver::put(hubo_ref_t* ref) {
  return ::put(&chan_hubo_ref, ref, "ref");
}

ach_status_t HuboAchDriver::get(hubo_state_t* state) {
  return ::get(&chan_hubo_state, state, "state");
}

ach_status_t HuboAchDriver::put(hubo_state_t* state) {
  return ::put(&chan_hubo_state, state, "state");
}
