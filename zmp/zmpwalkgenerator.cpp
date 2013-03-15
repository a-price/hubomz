#include "zmpwalkgenerator.h"

class ZMPWalkGenerator {
public:


    const HuboPlus& hplus;
    

    real com_height;
    // tons of constants

    ZMPReferenceContext initContext; 
    std::vector<ZMPReferenceContext> ref;

    std::vector<zmp_traj_element_t> traj; // the entire fullbody trajectory so far
    
    // INVARIANT: traj.back() agrees 100% with current

    // these all modify the current context but do not immediately affect traj

    // this needs to have everything realized
    void initialize(const ZMPReferenceContext& current) {
        ref.clear();
        traj.clear();

    }

    const ZMPReferenceContext& getLastRef() {
        return ref.empty() ? initContext : ref.back();
    }


    // these will add walk contexts to the back of ref and the new
    // contexts don't have comX, comY, eX, eY however, the kstate will
    // have body orientation set correctly and upper body joints
    void stayDogStay(size_t stay_ticks) {
       
        for (size_t i=0; i<stay_ticks; ++i) {
            ref.push_back(getLastRef());
        }

    }

    void addFootstep(const Footprint& fp) {
        
        // find out how long in double support
        size_t dticks = getdticks();

        stance_t dstance = getdoublestance();

        // not a C++ reference because I'm modifying ref inside loop
        const ZMPReferenceContext start = getLastRef();

        real pX_end = 0;
        real pY_end = 0;
        quat body_rot_end = quat();

        for (size_t i=0; i<dticks; ++i) {

            // sigmoidally interopolate things like desired ZMP and body rotation
            ref.push_back(start);

            ZMPReferenceContext& cur = ref.back();

            real u = real(i) / (dticks-1);
            real c = sigmoid(u);

            cur.stance = dstance;

            cur.pX = start.pX + c * (pX_end - start.pX);
            cur.pY = start.pY + c * (pY_end - start.pY);

            cur.state.body_rot = quat::slerp(start.state.body_rot, 
                                             body_rot_end, c);

        }

        size_t sticks = getsingle();

        stance_t sstance = getsinglestance();
        int swing_foot = getswing(sstance);
        int stance_foot = getstance(sstance);

        for (size_t i=0; sticks; ++i) {
            ref.push_back(getLastRef());
            ZMPReferenceContext& cur = ref.back();
            cur.feet[swing] = getfoottransformfromana(i);
        }

        
    }

    void bakeIt() {
        traj.clear();
        runZMPPreview();
        runCOMIK();
        dumpTraj();

        // for thinkin ahead
        initContext = ref[indexAfterFirstStep];
        ref.clear();

    }

private:

    // this runs the ZMP preview controller on the entire reference
    // trajectory to fill in comX, comY, eX, eY for every dang thing.
    void runZMPPreview() {

        /*
          todo: 
          initialize comX, comY, eX, eY from initContext
          pull out all pX, pY from ref into Eigen arrays
          run zmp preview on all dat shizzy 
        */

        // postcondition: now we have set comX, comY, eX, eY for everything in ref.

    }

    // this runs the COM IK on every dang thing in reference to fill
    // in the kstate
    void runCOMIK() {

        HuboPlus::IKMode mode[4] = { 
            HuboPlus::IK_MODE_FIXED, // lleg
            HuboPlus::IK_MODE_FIXED, // rleg
            HuboPlus::IK_MODE_FIXED, // larm
            HuboPlus::IK_MODE_FIXED, // rarm
        };

        Transform3 desired[4]; // same oprder

        for (size_t i=0; i<ref.size(); ++i) {
            
            ZMPReferenceContext& cur = ref[i];

            // todo: set mode based on cur.stance
            // stance foot gets support 
            // swing foot gets world

            // todo: set desired[0] = cur.foot[0] and etc for 1

            // pull the desired com oout of cur.comX[0], cur.comY[0], height
            vec3 desiredCom; 

            bool ok = hplus.comIK( desiredCom, ... );

            // TODO freak out if not ok

        }

        // postcondition: all of the kstates in each ref element are fully specified

    }

    // this dumps everything into traj
    void dumpTraj() {

        // pick everything important out of ref which is now fully specified
        // calculate desired forces and torques
        // transform everything into stance ankle reference frame

        
    }

};

void transforms101() {

    Transform3 leftFoot;

    vec3 pointInLeftFrame;

    vec3 pointInWorldFrame = leftFoot * pointInLeftFrame;
    pointInWorldFrame = leftFoot.transformFwd(pointInLeftFrame);

    vec3 velInLeftFrame;
    vec3 velInWorldFrame = leftFoot.rotFwd() * velInLeftFrame;
    
    pointInLeftFrame = leftFoot.transformInv(pointInWorldFrame); // this
    pointInLeftFrame = leftFoot.inverse() * pointInWorldFrame; // not this

    velInLeftFrame = leftFoot.rotInv() * velInWorldFrame;


    // this constructs the transform such that world = R(q)*obj + t
    Transform3 xform(q, t);

    q = xform.rotation();
    xform.setRotation(q);

    // ditto for translation


}
