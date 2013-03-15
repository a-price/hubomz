#include "math.h"
#include "mzcommon/Transform3.h"
#include "zmp/footprint.h"
#include "fakerave.h"
#include "hubo-zmp.h"
#include <HuboPlus.h>





using namespace fakerave;

class ZMPReferenceContext {
public:
    stance_t stance; // single/double, left/right

    Transform3 feet[2]; // or footprint i dont care
    Eigen::Vector3d comX, comY; // pos/vel/accel of each 
    double eX, eY; // integrator error for zmp controller
    double pX, pY; // where is the desired ZMP right now?
    
    HuboPlus::KState state; // complete state of the robot

};



class ZMPWalkGenerator {
public:
    
    ZMPWalkGenerator();
    
    const HuboPlus& hplus;
    double com_height;
    // tons of constants


    ZMPReferenceContext initContext; 
    std::vector<ZMPReferenceContext> ref;

    std::vector<zmp_traj_element_t> traj; // the entire fullbody trajectory so far
    
    // INVARIANT: traj.back() agrees 100% with current




    // these all modify the current context but do not immediately affect traj
    void initialize(const ZMPReferenceContext& current);
    const ZMPReferenceContext& getLastRef();
    // these will add walk contexts to the back of ref and the new
    // contexts don't have comX, comY, eX, eY however, the kstate will
    // have body orientation set correctly and upper body joints
    void stayDogStay(size_t stay_ticks);
    void addFootstep(const Footprint& fp);
    void bakeIt();
private:

    // this runs the ZMP preview controller on the entire reference
    // trajectory to fill in comX, comY, eX, eY for every dang thing.
    void runZMPPreview();
    // this runs the COM IK on every dang thing in reference to fill
    // in the kstate

    void runCOMIK();
    // this dumps everything into traj
    void dumpTraj();
};

// Local Variables:
// mode: c++
// End:
