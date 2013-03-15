#include "zmpwalkgenerator.h"

ZMPWalkGenerator::ZMPWalkGenerator(HuboPlus& _hplus) : hplus(_hplus) { }

// these all modify the current context but do not immediately affect traj

// this needs to have everything realized
void ZMPWalkGenerator::initialize(const ZMPReferenceContext& current) {
    ref.clear();
    traj.clear();

}

/**
* @function: ZMPReferenceContext& getLastRef()
* @brief: gets the last ZMPReferenceContext from ref or the initContext
*/
const ZMPWalkGenerator::ZMPReferenceContext& getLastRef() {
    return ref.empty() ? initContext : ref.back();
}


/**
* @function: stayDogStay(size_t stay_ticks)
* @brief: these will add walk contexts to the back of ref and the new
*         contexts don't have comX, comY, eX, eY however, the kstate will
*         have body orientation set correctly and upper body joints
* @return: void
*/
void ZMPWalkGenerator::stayDogStay(size_t stay_ticks) {
       
    for (size_t i=0; i<stay_ticks; ++i) {
        ref.push_back(getLastRef());
    }

}

void ZMPWalkGenerator::addFootstep(const Footprint& fp) {
        
    // find out how long in double support
    //TODO use eric's timer code
//    size_t dticks = getdticks();

    stance_t dstance = getdoublestance();

    // not a C++ reference because I'm modifying ref inside loop
    const ZMPReferenceContext start = getLastRef();

    double pX_end = 0;
    double pY_end = 0;
    quat body_rot_end = quat();

    for (size_t i=0; i<dticks; ++i) {

        // sigmoidally interopolate things like desired ZMP and body rotation
        ref.push_back(start);

        ZMPReferenceContext& cur = ref.back();

        double u = double(i) / (dticks-1);
        double c = sigmoid(u);

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
/**
* @function: bakeIt()
* @brief: clears a trajectory, runs ZMP Preview Controller
*         run COM IK, dumps out trajectory.
* @return: void
*/
void ZMPWalkGenerator::bakeIt() {
    traj.clear();
    runZMPPreview();
    runCOMIK();
    dumpTraj();

    // for thinkin ahead
    initContext = ref[indexAfterFirstStep];
    ref.clear();

}

private:

double ZMPWalkGenerator::sigmoid(double x) {
    return 3*x*x - 2*x*x*x;
}
/** @function: runZMPPreview()
* @brief: run ZMP preview controller on entire reference and creates trajectory for COM pos/vel/acc in X and Y
* @precondition: we have zmp reference values for x and y, initContext com and integrator error
*               for initialization.
* @postcondition: now we have set comX, comY, eX, eY for everything in ref.
* @return: void
*/
void ZMPWalkGenerator::runZMPPreview() {

    Eigen::Vector3d comX = initContext.comX; // initialize comX states to initial ZMPRefenceContext
    Eigen::Vector3d comY = initContext.comY; // initialize comY states to initial ZMPRefenceContext
    double eX = initContext.eX; // 
    double eY = initContext.eY; // 
    double pX = initContext.pX; // in
    double pY = initContext.pY; // where is the desired ZMP right now?
    Eigen::Vector3d X(0.0, 0.0, 0.0);
    Eigen::Vector3d Y(0.0, 0.0, 0.0);
    Eigen::ArrayXd zmprefX(ref.size()); // array for zmp x refs
    Eigen::ArrayXd zmprefY(ref.size()); // array for zmp y refs

    // put all the zmp refs into eigen arrays in order to pass into the preview controller
    for(size_t i=0; i<ref.size(); i++) {
        zmprefX(i) = ref[i].pX;
        zmprefY(i) = ref[i].pY;
    }

    // generate COM position for each tick using zmp preview update
    for(std::vector<ZMPReferenceContext>::iterator cur = ref.begin(); cur != ref.end(); cur++) {
        cur->comX = X.transpose(); // set the current ref comX pos/vel/acc
        cur->comY = Y.transpose(); // set the current ref comY pos/vel/acc
        cur->pX = pX; // set the current ref ZMPx
        cur->pY = pY; // set the current ref ZMPy

        // run zmp preview controller to update COM states and integrator error
        pX = preview.update(X, eX, zmprefX.block(i, 0, total_ticks-i, 1));
        pY = preview.update(Y, eY, zmprefY.block(i, 0, total_ticks-i, 1));
    }
}

/**
* @function: runCOMIK()
* @brief: this runs the COM IK on every dang thing in reference to fill in the kstate
* @precondition: reference is fully filled in
* @postcondition: kstate is fully filled in
* @return: void
*/
void ZMPWalkGenerator::runCOMIK() {

    std::vector<ZMPReferenceContext>* cur = &ref;

    Transform3Array xforms;
    const KinBody& kbody = hplus.kbody;
    HuboPlus::KState state;
    const double& l6 = hplus.footAnkleDist;

    Transform3 desired[4];
    vec3 desiredCom;

    HuboPlus::IKMode mode[4] = { 
        HuboPlus::IK_MODE_FIXED, // lleg
        HuboPlus::IK_MODE_FIXED, // rleg
        HuboPlus::IK_MODE_FIXED, // larm
        HuboPlus::IK_MODE_FIXED, // rarm
    };

    for (size_t t=0; t<ref.size(); t++) {
        // loop through stance and swing foot tables
        int stance_foot = stance_foot_table[cur[i]->stance];
        int swing_foot = swing_foot_table[cur[i]->stance];

        vec3 stanceFtPos = cur[i]->feet[stance_foot].translation();
        vec3 swingFtPos = cur[i]->feet[swing_foot].translation();
        
        vec3 old[2];
        for (int f=0; f<2; ++f) { old[f] = desired[f].translation(); }

        // if we're in double support mode
        if (swing_foot < 0) {
            mode[0] = HuboPlus::IK_MODE_SUPPORT;
            mode[1] = HuboPlus::IK_MODE_SUPPORT;
          
            desired[0].setTranslation(cur[i]->feet[0].translation()); // left foot x,y,z
            desired[1].setTranslation(cur[i]->feet[1].translation()); // right foot x,y,z

          // else if we're in swing mode
        } else {

            mode[swing_foot] = HuboPlus::IK_MODE_WORLD;
            mode[stance_foot] = HuboPlus::IK_MODE_SUPPORT;
            
            // set stance foot desired position to (0, fixed-pos from center, 0)
            desired[stance_foot].setTranslation(stanceFtPos);
            
            // set swing foot desired position equal to location set above for tick #i
            desired[swing_foot].setTranslation(swingFtPos);
        }
        // make sure change in foot position between two consecutive samples is less than limit 
        for (int f=0; t && f<2; ++f) {
            assert( (old[f] - desired[f].translation()).norm() < 0.05 );
        }
        
        // set com desired position
        vec3 desiredComTmp(desiredCom);
        desiredCom = vec3(cur[i]->comX(0), cur[i]->comY(0), com_height+l6);

        if ((desiredCom - desiredComTmp).norm() > .01 ) {
            assert( 0 && "Bad desiredCom" );
        }

        bool ikvalid[4];

        bool ok = hplus.comIK( state, desiredCom, desired, mode, 
            HuboPlus::noGlobalIK(), xforms, 
            com_ik_ascl, 0, ikvalid );

        // TODO freak out if not ok
        if (!ok) {

            Transform3 actual[4];
            vec3 dp[4], dq[4];

            kbody.transforms(state.jvalues, xforms);

            vec3 actualCom = state.xform() * kbody.com(xforms);

            // loop through legs and arms
            for (int i=0; i<4; ++i) {
                // if it's allowed to move
                if (mode[i] != HuboPlus::IK_MODE_FIXED &&
                    mode[i] != HuboPlus::IK_MODE_FREE) {
                    // get the transformations to each one
                    actual[i] = kbody.manipulatorFK(xforms, i);
                    //if it's in world of support mode
                    if (mode[i] == HuboPlus::IK_MODE_WORLD || 
                        mode[i] == HuboPlus::IK_MODE_SUPPORT) {
                        // get the actual transformation from ?? 
                        actual[i] = state.xform() * actual[i];
                    }

                    deltaTransform(desired[i], actual[i], dp[i], dq[i]);

                    // TODO: allow feet far away from ground to have IK failures
                }

            }
            // if IK not okay display errors
            if (!ok) {
                std::cerr << "IK FAILURE!\n\n";
                std::cerr << "  body:        " << state.xform() << "\n";
                std::cerr << "  desired com: " << desiredCom << "\n";
                std::cerr << "  actual com:  " << actualCom << "\n\n";
                for (int i=0; i<4; ++i) { 
                    if (mode[i] != HuboPlus::IK_MODE_FIXED &&
                        mode[i] != HuboPlus::IK_MODE_FREE) {
                        std::cerr << "  " << kbody.manipulators[i].name << ":\n";
                        std::cerr << "    valid:   " << ikvalid[i] << "\n";
                        std::cerr << "    desired: " << desired[i] << "\n";
                        std::cerr << "    actual:  " << actual[i] << "\n";
                        std::cerr << "    dp:      " << dp[i] << " with norm " << dp[i].norm() << "\n";
                        std::cerr << "    dq:      " << dq[i] << " with norm " << dq[i].norm() << "\n";
                        std::cerr << "\n";
                    }
                }
            exit(1);
            }
        }
    }
}

/**
* @function: dumpTraj()
* @brief: picks everything important out of ref which is now fully specified and creates a trajectory 
* @precondition: we have ref fully filled in
* @postcondition: forces and torque are calculated and everything is transformed into stance ankle reference frame
* @return: void
*/
void ZMPWalkGenerator::dumpTraj() {

    HuboPlus::IKMode mode[4] = { 
      HuboPlus::IK_MODE_FIXED,
      HuboPlus::IK_MODE_FIXED,
      HuboPlus::IK_MODE_FIXED,
      HuboPlus::IK_MODE_FIXED,
    };

    // loop through entire reference struct
    for (size_t i=0; i<ref.size(); i++) {
        zmp_traj_element_t cur; // new zmp_traj_element for each loop iteration
        memset(&cur, 0, sizeof(cur)); // clear it

        // loop through joints for Hubo+ model
        for (size_t hi=0; hi < hplus.huboJointOrder.size(); ++hi) {
            size_t ji = hplus.huboJointOrder[hi]; // get corresponding joint number for real Hubo
            if (ji != size_t(-1)) { // if it's a valid joint number
                cur.angles[hi] = state.jvalues[ji]; // set its angle to whatever is in state
            }
            cur.stance = ref[i].stance;
        }
        // get transformation from stance foot to world origin
        Transform3 stanceInv = desired[stance_foot].inverse();
        vec3 zmp(ref[i].pX, ref[i].pY, 0); // zmp vector for current tick
        zmp = stanceInv * zmp; // zmp transformation in stance foot frame

        cur.zmp[0] = zmp[0]; // add zmp x to current trajectory tick
        cur.zmp[1] = zmp[1]; // add zmp y to current trajectory tick

        vec3 forces[2], torques[2]; // create forces and torque vectors for each ankle

        // compute ground reaction forces and torques
        hplus.computeGroundReaction( vec3(ref[i].comX(0), ref[i].comY(0), com_height),
                     vec3(ref[i].comX(2), ref[i].comY(2), 0),
                     desired, mode,
                     forces, torques );

        // for each ankle set current trajectory force and torque vectors
        for (int f=0; f<2; ++f) {
            for (int axis=0; axis<3; ++axis) {
                cur.forces[f][axis] = forces[f][axis];
                cur.torque[f][axis] = torques[f][axis]; // TODO: FIXME: pluralization WTF?
            }
        }

        // get COM pos/vel/acc with respect to the stance foot frame
        for (int deriv=0; deriv<3; ++deriv) {
            vec3 cv(ref[i].comX(deriv), ref[i].comY(deriv), deriv==0 ? com_height : 0);
            if (deriv == 0) { // if it's position
                cv = stanceInv * cv;
            } else { // else if it's velocity or acceleration
                cv = stanceInv.rotFwd() * cv;
            }
            for (int axis=0; axis<3; ++axis) { // for each axis set new COM state w.r.t. stance foot
                cur.com[axis][deriv] = cv[axis];
            }
        }

        traj.push_back(cur); // add current trajectory tick to trajectory
}

/*void transforms101() {

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


}*/
