#include "ZmpPreview.h"
#include "HuboPlus.h"
#include "zmpwalkgenerator.h"
#include "gait-timer.h"

ZMPWalkGenerator::ZMPWalkGenerator(HuboPlus& _hplus,
                                   double com_height,
                                   double zmp_R,
                                   double com_ik_ankle_weight,
                                   double min_single_support_time,
                                   double min_double_support_time,
                                   double walk_startup_time,
                                   double walk_shutdown_time) :
    hplus(_hplus),
    com_height(com_height),
    zmp_R(zmp_R),
    com_ik_ankle_weight(com_ik_ankle_weight),
    min_single_support_time(min_single_support_time),
    min_double_support_time(min_double_support_time),
    walk_startup_time(walk_startup_time),
    walk_shutdown_time(walk_shutdown_time)
{
}

// these all modify the current context but do not immediately affect traj

void ZMPWalkGenerator::initialize(const ZMPReferenceContext& current) {
    ref.clear();
    traj.clear();
}

/**
 * @function: ZMPReferenceContext& getLastRef()
 * @brief: gets the last ZMPReferenceContext from ref or the initContext
 */
const ZMPReferenceContext& ZMPWalkGenerator::getLastRef() {
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


double ZMPWalkGenerator::sigmoid(double x) {
    return 3*x*x - 2*x*x*x;
}

void ZMPWalkGenerator::addFootstep(const Footprint& fp) {
    
    GaitTimer timer;
    timer.single_support_time = min_single_support_time;
    timer.double_support_time = min_double_support_time;
    timer.startup_time = walk_startup_time;
    timer.startup_time = walk_shutdown_time;

    
    
    // // find out how long in double support
    // //TODO use eric's timer code
    // size_t dticks = getdticks();

    // stance_t dstance = getdoublestance();

    // // not a C++ reference because I'm modifying ref inside loop
    // const ZMPReferenceContext start = getLastRef();

    // double pX_end = 0;
    // double pY_end = 0;
    // quat body_rot_end = quat();

    // for (size_t i=0; i<dticks; ++i) {

    //     // sigmoidally interopolate things like desired ZMP and body rotation
    //     ref.push_back(start);

    //     ZMPReferenceContext& cur = ref.back();

    //     double u = double(i) / (dticks-1);
    //     double c = sigmoid(u);

    //     cur.stance = dstance;

    //     cur.pX = start.pX + c * (pX_end - start.pX);
    //     cur.pY = start.pY + c * (pY_end - start.pY);

    //     cur.state.body_rot = quat::slerp(start.state.body_rot, 
    //                                      body_rot_end, c);

    // }

    // size_t sticks = getsingle();

    // stance_t sstance = getsinglestance();
    // int swing_foot = getswing(sstance);
    // int stance_foot = getstance(sstance);

    // for (size_t i=0; sticks; ++i) {
    //     ref.push_back(getLastRef());
    //     ZMPReferenceContext& cur = ref.back();
    //     cur.feet[swing] = getfoottransformfromana(i);
    // }

        
}
/**
 * @function: bakeIt()
 * @brief: clears a trajectory, runs ZMP Preview Controller
 *         run COM IK, dumps out trajectory.
 * @return: void
 */
void ZMPWalkGenerator::bakeIt() {
    // traj.clear();
    // runZMPPreview();
    // runCOMIK();
    // dumpTraj();

    // // for thinkin ahead
    // initContext = ref[indexAfterFirstStep];
    // ref.clear();

}

/** @function: runZMPPreview()
 * @brief: run ZMP preview controller on entire reference and creates trajectory for COM pos/vel/acc in X and Y
 * @precondition: we have zmp reference values for x and y, initContext com and integrator error
 *               for initialization.
 * @postcondition: now we have set comX, comY, eX, eY for everything in ref.
 * @return: void
 */
void ZMPWalkGenerator::runZMPPreview() {

    Eigen::Vector3d comX = initContext.comX; // initialize comX states to initial ZMPReferenceContext
    Eigen::Vector3d comY = initContext.comY; // initialize comY states to initial ZMPReferenceContext
    double eX = initContext.eX; // 
    double eY = initContext.eY; // 
    Eigen::ArrayXd zmprefX(ref.size());
    Eigen::ArrayXd zmprefY(ref.size());

    // initialize the zmp preview controller
    ZmpPreview preview(1.0/TRAJ_FREQ_HZ, com_height, ref.size(), zmp_R);

    // put all the zmp refs into eigen arrays in order to pass into the preview controller
    for(size_t i=0; i<ref.size(); i++) {
        zmprefX(i) = ref[i].pX;
        zmprefY(i) = ref[i].pY;
    }

    // generate COM position for each tick using zmp preview update
    for(size_t i = 0; i < ref.size(); i++) {
        ZMPReferenceContext& cur = ref[i];
        // run zmp preview controller to update COM states and integrator error
        preview.update(comX, eX, zmprefX.rightCols(zmprefX.size()-i));
        preview.update(comY, eY, zmprefY.rightCols(zmprefX.size()-i));
        cur.comX = comX.transpose(); // set the ref comX pos/vel/acc for this tick
        cur.comY = comY.transpose(); // set the ref comY pos/vel/acc for this tick
        cur.eX = eX;                 // set the X error for this tick
        cur.eY = eY;                 // set the Y error for this tick
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
    vec3 desiredCom;

    // initialize transforms from initial hubo configuration
    Transform3Array body_transforms;
    hplus.kbody.transforms(initContext.state.jvalues, body_transforms);
        

    for(std::vector<ZMPReferenceContext>::iterator cur = ref.begin(); cur != ref.end(); cur++) {
        // set up target positions
        Transform3 desired[4] = {cur->feet[0], cur->feet[1], Transform3(), Transform3()};
        desiredCom = vec3(cur->comX[0], cur->comY[0], com_height - hplus.footAnkleDist);
        // My guess, and why I changed it from addition to
        // subtraction: the bottom of the foot is footAnkleDist away
        // from the "end" of the end effector, so to get our actual
        // desired com_height we have to set our desired end effector
        // position a bit closer to the com than we would otherwise.
        // TODO: this needs to be checked by someone that actually
        // knows what they're talking about.

        // set up mode
        // single right is the only time the left foot is raised and vice versa
        // TODO: find a way to cut out the HuboPlus junk, it's unsightly
        cur->ikMode[HuboPlus::MANIP_L_FOOT] = (cur->stance == SINGLE_RIGHT) ? HuboPlus::IK_MODE_WORLD : HuboPlus::IK_MODE_SUPPORT;
        cur->ikMode[HuboPlus::MANIP_R_FOOT] = (cur->stance == SINGLE_LEFT) ? HuboPlus::IK_MODE_WORLD : HuboPlus::IK_MODE_SUPPORT;
        cur->ikMode[HuboPlus::MANIP_L_HAND] = HuboPlus::IK_MODE_FIXED;
        cur->ikMode[HuboPlus::MANIP_R_HAND] = HuboPlus::IK_MODE_FIXED;
        
        // set up ikvalid
        bool ikvalid[4];
        
        // and run IK. Everything we need goes straight into cur->state! cool.
        bool ok = hplus.comIK(cur->state,
                              desiredCom,
                              desired,
                              cur->ikMode, 
                              HuboPlus::noGlobalIK(),
                              body_transforms, 
                              com_ik_ankle_weight,
                              0,
                              ikvalid);

        // TODO freak out if not ok
        if (!ok) {
            Transform3 actual[4];
            vec3 dp[4], dq[4];
            hplus.kbody.transforms(cur->state.jvalues, body_transforms);
            vec3 actualCom = cur->state.xform() * hplus.kbody.com(body_transforms);

            // loop through legs and arms to calculate results
            for (int i=0; i<4; ++i) {
                // if it's allowed to move
                if (cur->ikMode[i] != HuboPlus::IK_MODE_FIXED &&
                    cur->ikMode[i] != HuboPlus::IK_MODE_FREE) {
                    // get the transformations to each one
                    actual[i] = hplus.kbody.manipulatorFK(body_transforms, i);
                    //if it's in world of support cur->ikMode
                    if (cur->ikMode[i] == HuboPlus::IK_MODE_WORLD || 
                        cur->ikMode[i] == HuboPlus::IK_MODE_SUPPORT) {
                        // get the actual transformation from ?? 
                        actual[i] = cur->state.xform() * actual[i];
                    }

                    deltaTransform(desired[i], actual[i], dp[i], dq[i]);

                    // TODO: allow feet far away from ground to have IK failures
                }
            }

            std::cerr << "IK FAILURE!\n\n";
            std::cerr << "  body:        " << cur->state.xform() << "\n";
            std::cerr << "  desired com: " << desiredCom << "\n";
            std::cerr << "  actual com:  " << actualCom << "\n\n";
            for (int i=0; i<4; ++i) { 
                if (cur->ikMode[i] != HuboPlus::IK_MODE_FIXED &&
                    cur->ikMode[i] != HuboPlus::IK_MODE_FREE) {
                    std::cerr << "  " << hplus.kbody.manipulators[i].name << ":\n";
                    std::cerr << "    valid:   " << ikvalid[i] << "\n";
                    std::cerr << "    desired: " << desired[i] << "\n";
                    std::cerr << "    actual:  " << actual[i] << "\n";
                    std::cerr << "    dp:      " << dp[i] << " with norm " << dp[i].norm() << "\n";
                    std::cerr << "    dq:      " << dq[i] << " with norm " << dq[i].norm() << "\n";
                    std::cerr << "\n";
                }
            }
            exit(1);
        } // finish freaking out

        // and we're done!
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
    for(std::vector<ZMPReferenceContext>::iterator cur_ref = ref.begin(); cur_ref != ref.end(); cur_ref++) {
        // make output
        zmp_traj_element_t cur_out;
        
        // copy stance into output
        cur_out.stance = cur_ref->stance;
        
        // copy joint angles from reference to output
        for (size_t sim_i=0; sim_i < hplus.huboJointOrder.size(); sim_i++) {
            // map simulation joint number to physical joint number
            size_t phys_i = hplus.huboJointOrder[sim_i];
            if (phys_i != size_t(-1)) { // if it's valid, copy
                cur_out.angles[phys_i] = cur_ref->state.jvalues[sim_i];
            }
        }
        
        // compute expected forces and torques and copy into output
        vec3 forces[2];
        vec3 torques[2];
        Transform3 desired[4] = {cur_ref->feet[0], cur_ref->feet[1], Transform3(), Transform3()};
        // compute ground reaction forces and torques
        hplus.computeGroundReaction( vec3(cur_ref->comX(0), cur_ref->comY(0), com_height), // com position, we hope
                                     vec3(cur_ref->comX(2), cur_ref->comY(2), 0), // com acceleration
                                     desired, // foot locations
                                     cur_ref->ikMode, // foot modes
                                     forces,  // forces output
                                     torques); // torques output
        for (size_t axis=0; axis<3; axis++) {  // here's one of the places we have to swap handedness
            cur_out.forces[0][axis] = forces[1][axis]; // left foot forces
            cur_out.torque[0][axis] = torques[1][axis]; // left foot torques
            cur_out.forces[1][axis] = forces[0][axis];  // right foot forces
            cur_out.torque[1][axis] = torques[0][axis]; // right foot torques
        }

        // transform zmp and com into stance ankle reference frame, copy into output
        Transform3 stance_foot_trans = cur_ref->stance == SINGLE_LEFT || cur_ref->stance == DOUBLE_LEFT
            ? cur_ref->feet[0] : cur_ref->feet[1];
        vec3 zmp_in_world(cur_ref->pX, cur_ref->pY, 0);
        vec3 zmp_in_stance = stance_foot_trans.transformInv(zmp_in_world);
        cur_out.zmp[0] = zmp_in_stance.x();
        cur_out.zmp[1] = zmp_in_stance.y();
        for (size_t deriv = 0; deriv < 3; deriv++) {
            vec3 com_in_world(cur_ref->comX[0], cur_ref->comY[0], deriv == 0 ? com_height : 0);
            vec3 com_in_stance = stance_foot_trans.transformInv(com_in_world);
            cur_out.com[0][deriv] = com_in_stance.x();
            cur_out.com[1][deriv] = com_in_stance.y();
            cur_out.com[2][deriv] = com_in_stance.z();
        }

        traj.push_back(cur_out); // add the output to our finished product
        // and we're done!
    }
}
