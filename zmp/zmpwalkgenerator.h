#ifndef ZMPWALKGENERATOR_H
#define ZMPWALKGENERATOR_H

#include "math.h"
#include "mzcommon/Transform3.h"
#include "zmp/footprint.h"
#include "fakerave.h"
#include "hubo-zmp.h"
#include "gait-timer.h"
#include "HuboPlus.h"
#include "zmp-daemon.h"




using namespace fakerave;





class ZMPReferenceContext {
public:
    stance_t stance; // single/double, left/right
    HuboPlus::IKMode ikMode[4]; // current IK settings for each limb

    Transform3 feet[2]; // or footprint i dont care
    Eigen::Vector3d comX, comY; // pos/vel/accel of each 
    double eX, eY; // integrator error for zmp controller
    double pX, pY; // where is the desired ZMP right now?
    
    HuboPlus::KState state; // complete state of the robot
};

class ZMPWalkGenerator {
public:
    
  // TODO: shove these into struct
  


    ZMPWalkGenerator(HuboPlus& _hplus,
        		     ik_error_sensitivity ik_sense,
                     double com_height,
                     double zmp_R,
		             double zmpoff_x,
		             double zmpoff_y,
                     double com_ik_angle_weight,
                     double min_single_support_time,
                     double min_double_support_time,
                     double walk_startup_time,
                     double walk_shutdown_time,
                     double step_height,
		             double lookahead_time);
    
    const HuboPlus& hplus;

    ik_error_sensitivity ik_sense;
    double com_height;
    double zmp_R; // jerk penalty on ZMP controller
    double zmpoff_x;
    double zmpoff_y;
    double com_ik_angle_weight;
    double min_single_support_time;
    double min_double_support_time;
    double walk_startup_time;
    double walk_shutdown_time;
    double step_height;
    double lookahead_time;
    // tons of constants

    size_t first_step_index;
    
    bool haveInitContext;

    // initial zmp data structure
    ZMPReferenceContext initContext; 

    // array of all zmp data structure for trajectory
    std::vector<ZMPReferenceContext> ref;
    size_t startTick;

    // array fullbody joint trajectory at each tick in previous and current traj 
    std::vector<zmp_traj_element_t> traj;
    std::vector<zmp_traj_element_t> prevTraj;
    
    // INVARIANT: traj.back() agrees 100% with current

    // these all modify the current context but do not immediately affect traj
    void initialize(const ZMPReferenceContext& current);

    /**
     * @function: ZMPReferenceContext& getLastRef()
     * @brief: gets the last ZMPReferenceContext from ref or the initContext
     */
    const ZMPReferenceContext& getLastRef();

    /**
     * @function: stayDogStay(size_t stay_ticks)
     * @brief: these will add walk contexts to the back of ref and the new
     *         contexts don't have comX, comY, eX, eY however, the kstate will
     *         have body orientation set correctly and upper body joints
     * @return: void
    */
    void stayDogStay(size_t stay_ticks);

    /**
     * @function: addFootstep(const Footprint& fp);
     * @brief: add a Footprint to the current ZMPReferenceContext array
    */
    void addFootstep(const Footprint& fp);

    /**
     * @function: bakeIt()
     * @brief: clears a trajectory, runs ZMP Preview Controller
     *         run COM IK, dumps out trajectory.
     * @return: void
    */
    void bakeIt();
 
    /**
     * @function: clearRef()
     * @brief: clears the ref trajectory
    */
    void clearRef();

    /**
     * @function:  applyComIK(ZMPReferenceContext &ref)
     * @brief: runs the runComIK function on entire array of contexts
    */
    void applyComIK(ZMPReferenceContext& ref);

    /**
     * @function: refToTraj(const ZMPReferenceContext &ref)
     * @brief: converts the reference trajectory to a trajectory
     * that can be used by Hubo, which doesn't have a world origin.
    */
    void refToTraj(const ZMPReferenceContext& ref,
		   zmp_traj_element_t& traj);


    
    // helper
    double sigmoid(double x);
    
    // this runs the ZMP preview controller on the entire reference
    // trajectory to fill in comX, comY, eX, eY for every dang thing.
    void runZMPPreview();

    /**
     * @function: runCOMIK()
     * @brief: this runs the COM IK on every dang thing in reference to fill in the kstate
     * @precondition: reference is fully filled in
     * @postcondition: kstate is fully filled in
     * @return: void
    */
    void runCOMIK();

    /**
     * @function: dumpTraj()
     * @brief: picks everything important out of ref which is now fully specified and creates a trajectory
     * @precondition: we have ref fully filled in
     * @postcondition: forces and torque are calculated and everything is transformed into stance ankle reference frame
     * @return: void
     */
    void dumpTraj();

    /**
     * @function: getNextInitContext()
     * @brief: grabs the intitial ZMPReferenceContext
     * for the next trajectory, which is at the end
     * of the last step
    */
    ZMPReferenceContext getNextInitContext(int tick);

    /**
     * @function: getStartTick()
     * @brief: gets the startTick for the next trajectory
     * @precondition: need to have just called getNextInitContext()
     * @return: returns the size_t startTick
    */
    size_t getStartTick();

    /**
     * @function: getRefTraj()
     * @brief: gets the full ref trajectory
     * as a vector of ZMPReferenceContext which
     * are in the world frame
    */
    std::vector<ZMPReferenceContext> getRefTraj();
};

// Local Variables:
// mode: c++
// End:

#endif // ZMPWALKGENERATOR_H
