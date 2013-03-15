
















struct ZMPReferenceContext {

    stance_t stance; // single/double, left/right

    Transform3 feet[2]; // or footprint i dont care
    Eigen::Vector3d comX, comY; // pos/vel/accel of each 
    real eX, eY; // integrator error for zmp controller
    real pX, pY; // where is the desired ZMP right now?
    
    HuboPlus::KState state; // complete state of the robot

};


