/********************************************************/
/*    Author: Matt Zucker <mzucker1@swarthmore.edu>     */
/* Copyright (c) 2012 Matt Zucker. All rights reserved. */
/********************************************************/

#include "HuboPlus.h"
#include <mzcommon/glstuff.h>
#include <mzcommon/TimeUtil.h>

#define assert_equal(x, y) _assert_equal((x), (y), #x, #y, __FILE__, __LINE__)





using namespace fakerave;
using namespace HK;

static inline void _assert_equal(real x, real y, 
                                 const char* xstr, 
                                 const char* ystr,
                                 const char* file,
                                 int line) {
  if (fabs(x-y) > 1e-12) {
    std::cerr << file << ":" << line << ": " 
              << xstr << "=" << x << ", " 
              << ystr << "=" << y << "\n";
    abort();
  }
}

const char* mnames[4] = {
  "leftFootManip",
  "rightFootManip",
  "leftHandManip",
  "rightHandManip",
};

std::string flipy(const std::string& name) {
  size_t idx = name.find('L');
  if (idx != std::string::npos) {
    std::string n = name;
    n[idx] = 'R';
    return n;
  } else {
    return name;
  }
}

Transform3 flipy(const Transform3& t) {

  mat4_t<real> m = mat4_t<real>::identity();
  m(1,1) = -1;

  return Transform3(m * t.matrix() * m);

}

vec3 flipy(const vec3& v) {
  return vec3(v.x(), -v.y(), v.z());
}

void compare(const vec3& l, const vec3& r) {
  assert( (l - r).norm() < 1e-8 );
}

void compare(const Transform3& l, const Transform3& r) {
  vec3 dp, dq;
  deltaTransform(l, r, dp, dq);
  assert( dp.norm() < 1e-8 && dq.norm() < 1e-8 );
}

void ensureMirror(const Joint& jl, const Joint& jr) {

  vec3 al = jl.anchor;
  vec3 ar = jr.anchor;

  compare(al, flipy(ar));
  compare(jl.axis, jr.axis);

  if (!jl.axis.y()) {
    if (jl.limits[0] < jl.limits[1]) {
      assert(jl.limits[0] == -jr.limits[1]);
      assert(jl.limits[1] == -jr.limits[0]);
    }
  }

}

void ensureMirror(const Body& bl, const Body& br) {

  Transform3 xl = bl.xform;
  Transform3 xr = br.xform;

  compare(xl, flipy(xr));

}

class AnchorLookup {
public:
  const KinBody& kbody;
  const Transform3Array& xforms;
  AnchorLookup(const KinBody& k, const Transform3Array& x): kbody(k), xforms(x) {}
  vec3 operator()(const std::string& n) {
    return kbody.jointAnchor(xforms, kbody.lookupJoint(n));
  }
};

HuboPlus::HuboPlus(const std::string& filename): 
  bl(kbody), jl(kbody), ml(kbody)

{
  
  DEFAULT_COM_ITER = 10000;
  DEFAULT_COM_PTOL = 1e-3;

  kbody.loadXML(filename);

  for (size_t i=0; i<4; ++i) {
    assert( ml(mnames[i]) == i );
  }

  for (size_t j=0; j<kbody.joints.size(); ++j) {
    const Joint& jleft = kbody.joints[j];
    size_t jj = jl(flipy(jleft.name));
    if (jj != j && jj != size_t(-1)) {
      const Joint& jright = kbody.joints[jj];
      ensureMirror(jleft, jright);
    }
  }

  for (size_t b=0; b<kbody.bodies.size(); ++b) {
    const Body& bleft = kbody.bodies[b];
    size_t bb = bl(flipy(bleft.name));
    if (bb != b && bb != size_t(-1)) {
      const Body& bright = kbody.bodies[bb];
      ensureMirror(bleft, bright);
    }
  }

  Transform3Array xforms;
  RealArray jvalues(kbody.joints.size(), 0.0);

  kbody.transforms(jvalues, xforms);

  defaultFootPos = kbody.manipulatorFK(xforms, 0).translation();
  defaultComPos = kbody.com(xforms);

  // now set some lengths and stuff
  HK::HuboKin::KinConstants& kc = hkin.kc;

  AnchorLookup ja(kbody, xforms);

  // FIX
  kbody.alignJoint(jl("LAR"), xforms, ja("LHR"), vec3(0,1,0), false);
  kbody.alignJoint(jl("RAR"), xforms, ja("RHR"), vec3(0,1,0), false);
  //kbody.alignJoint(jl("HPY"), xforms, vec3(0), vec3(0,1,0), false);
  
  assert_equal( ja("LHY").y(), ja("LHR").y() );
  assert_equal( ja("LHY").y(), ja("LAR").y() );
  assert_equal( ja("LHY").x(), ja("LHP").x() );
  assert_equal( ja("LHY").x(), ja("LKP").x() );
  assert_equal( ja("LHY").x(), ja("LAP").x() );

  kbody.alignJoint(jl("LSP"), xforms, ja("LSR"), vec3(0,1,0), true);
  kbody.alignJoint(jl("LSR"), xforms, ja("LSP"), vec3(1,0,0), true);
  kbody.alignJoint(jl("LSY"), xforms, ja("LSP"), vec3(0,0,1), true);
  kbody.alignJoint(jl("LEP"), xforms, ja("LSR"), vec3(0,1,0), true);
  kbody.alignJoint(jl("LWY"), xforms, ja("LWP"), vec3(0,0,1), true);
  kbody.alignJoint(jl("LWP"), xforms, ja("LSR"), vec3(0,1,0), true);

  kbody.alignJoint(jl("RSP"), xforms, ja("RSR"), vec3(0,1,0), true);
  kbody.alignJoint(jl("RSR"), xforms, ja("RSP"), vec3(1,0,0), true);
  kbody.alignJoint(jl("RSY"), xforms, ja("RSP"), vec3(0,0,1), true);
  kbody.alignJoint(jl("REP"), xforms, ja("RSR"), vec3(0,1,0), true);
  kbody.alignJoint(jl("RWY"), xforms, ja("RWP"), vec3(0,0,1), true);
  kbody.alignJoint(jl("RWP"), xforms, ja("RSR"), vec3(0,1,0), true);

  kbody.alignJoint(jl("LHY"), xforms, ja("LHR"), vec3(0,0,1), true);
  kbody.alignJoint(jl("LHR"), xforms, ja("LHY"), vec3(1,0,0), true);
  kbody.alignJoint(jl("LHP"), xforms, ja("LHY"), vec3(0,1,0), true);
  kbody.alignJoint(jl("LKP"), xforms, ja("LHY"), vec3(0,1,0), true);
  kbody.alignJoint(jl("LAP"), xforms, ja("LHY"), vec3(0,1,0), true);
  kbody.alignJoint(jl("LAR"), xforms, ja("LHY"), vec3(1,0,0), true);

  kbody.alignJoint(jl("RHY"), xforms, ja("RHR"), vec3(0,0,1), true);
  kbody.alignJoint(jl("RHR"), xforms, ja("RHY"), vec3(1,0,0), true);
  kbody.alignJoint(jl("RHP"), xforms, ja("RHY"), vec3(0,1,0), true);
  kbody.alignJoint(jl("RKP"), xforms, ja("RHY"), vec3(0,1,0), true);
  kbody.alignJoint(jl("RAP"), xforms, ja("RHY"), vec3(0,1,0), true);
  kbody.alignJoint(jl("RAR"), xforms, ja("RHY"), vec3(1,0,0), true);

  // Are joint offsets contributing to disagreement between FK's?
  // It seems like not...
  if (0)  {
    for (size_t ji=0; ji<kbody.joints.size(); ++ji) {
      kbody.transforms(jvalues, xforms);
      vec3 anchor = kbody.jointAnchor(xforms, ji);
      size_t bi = kbody.joints[ji].body2Index;
      vec3 origin = xforms[bi].translation();
      vec3 diff = anchor - origin;
      kbody.offsetBody(bi, diff);
    }
  }

  kbody.transforms(jvalues, xforms);

  for (size_t j=0; j<kbody.joints.size(); ++j) {
    const Joint& jleft = kbody.joints[j];
    size_t jj = jl(flipy(jleft.name));
    if (jj != j && jj != size_t(-1)) {
      const Joint& jright = kbody.joints[jj];
      ensureMirror(jleft, jright);
    }
  }

  for (size_t b=0; b<kbody.bodies.size(); ++b) {
    const Body& bleft = kbody.bodies[b];
    size_t bb = bl(flipy(bleft.name));
    if (bb != b && bb != size_t(-1)) {
      const Body& bright = kbody.bodies[bb];
      ensureMirror(bleft, bright);
    }
  }


  kc.leg_l1 = ja("LSP").z() - ja("HPY").z();
  kc.leg_l2 = ja("LHY").y();
  kc.leg_l3 = ja("HPY").z() - ja("LHR").z();
  kc.leg_l4 = ja("LHR").z() - ja("LKP").z();
  kc.leg_l5 = ja("LKP").z() - ja("LAR").z();
  kc.leg_l6 = ja("LAR").z() - defaultFootPos.z();

  // handle the limits 
  const IndexArray& jidx = kbody.manipulators[1].jointIndices;
  for (size_t i=0; i<jidx.size(); ++i) {
    real& lo = kc.leg_limits(i,0);
    real& hi = kc.leg_limits(i,1);
    const Joint& j = kbody.joints[jidx[i]];
    if (j.limits[0] < j.limits[1]) {
      lo = j.limits[0];
      hi = j.limits[1];
    } else {
      lo = -M_PI/2;
      hi = M_PI/2;
    }
  }

  real jy = kc.leg_l2;
  real jz = kc.leg_l1 + kc.leg_l3 + kc.leg_l4 + kc.leg_l5 + kc.leg_l6;

  size_t lfoot = bl("Body_LAR");
  size_t rfoot = bl("Body_RAR");
  vec3 t0 = xforms[lfoot].transformInv(vec3(0, jy, -jz));

  real s2 = sqrt(2)/2;
  footRot = quat(0, s2, 0, s2);

  kbody.manipulators[0].xform = Transform3(t0);

  t0 = xforms[rfoot].transformInv(vec3(0, -jy, -jz));
  kbody.manipulators[1].xform = Transform3(t0);

}


void HuboPlus::render(const Transform3Array& xforms,
                      const Vec4Array* overrideColors) const {
  
  kbody.render(xforms,
               vec4(0.5,0.5,0.5,1), 
               overrideColors);

}

bool HuboPlus::manipIK(size_t mi,
                       const Transform3& desired,
                       RealArray& jvalues,
                       Transform3Array& xforms,
                       bool global) const {

  if (global && (mi == MANIP_L_FOOT || mi == MANIP_R_FOOT)) { 
    
    Transform3 dd = desired * Transform3(footRot);
    mat4_t<real> m = dd.matrix();
    Isometry3d B;

    for (int i=0; i<4; ++i) { 
      for (int j=0; j<4; ++j) {
        B(i,j) = m(i,j);
      }
    }

    const Manipulator& manip = kbody.manipulators[mi];
    const IndexArray& jidx = manip.jointIndices;

    Vector6d qprev, q;
    stdvec2mat(jvalues, qprev, jidx);
    
    int side = (mi == MANIP_L_FOOT ? HuboKin::LEFT : HuboKin::RIGHT);
    
    hkin.legIK(q, B, qprev, side);

    mat2stdvec(q, jvalues, jidx);
    

    vec3 dp, dq;

    if (1) {

      kbody.transforms(jvalues, xforms, &manip.activeBodies);
      Transform3 fk = kbody.manipulatorFK(xforms, mi);
      deltaTransform(desired, fk, dp, dq);
      
    } else {

      hkin.legFK(B, q, side);

      for (int i=0; i<4; ++i) { 
        for (int j=0; j<4; ++j) {
          m(i,j) = B(i, j);
        }
      }

      deltaTransform(dd, Transform3(m), dp, dq);

    }
    //std::cout << "dp.norm() = " << dp.norm() << "\n";
    //std::cout << "dq.norm() = " << dq.norm() << "\n";

    return (dp.norm() <= kbody.DEFAULT_PTOL && dq.norm() <= kbody.DEFAULT_QTOL);

  } else {
    
    if (global) { initIK(mi, desired, jvalues); }
    return kbody.manipulatorIK(mi, desired, jvalues, xforms);

  }

}

void HuboPlus::initIK(size_t mi,
                      const Transform3& desired,
                      RealArray& jvalues) const {

  const real deg = M_PI/180;
  const real lang = 30*deg;


  if (mi == MANIP_L_FOOT) {
    jvalues[ jl("LHP") ] = -lang;
    jvalues[ jl("LKP") ] = 2*lang;
    jvalues[ jl("LAP") ] = -lang;
  } else if (mi == MANIP_R_FOOT) {
    jvalues[ jl("RHP") ] = -lang;
    jvalues[ jl("RKP") ] = 2*lang;
    jvalues[ jl("RAP") ] = -lang;
  } else if (mi == MANIP_L_HAND) {
    jvalues[ jl("LSP") ] = lang;
    jvalues[ jl("LEP") ] = -2*lang;
    jvalues[ jl("LWP") ] = lang;
  } else if (mi == MANIP_R_HAND) {
    jvalues[ jl("RSP") ] = lang;
    jvalues[ jl("REP") ] = -2*lang;
    jvalues[ jl("RWP") ] = lang;
  } else {
    kbody.centerJoints(kbody.manipulators[mi].jointIndices, jvalues);
  }
  
}


bool HuboPlus::stanceIK( KState& state,
                              const Transform3 manipXforms[NUM_MANIPULATORS],
                              const IKMode mode[NUM_MANIPULATORS],
                              const bool shouldInitIK[NUM_MANIPULATORS],
                              Transform3Array& work,
                              bool* ikvalid ) const {

  bool allOK = true;

  kbody.transforms(state.jvalues, work);

  for (int i=0; i<NUM_MANIPULATORS; ++i) {

    Transform3 desired = manipXforms[i];
    bool doIK = false;
    
    switch (mode[i]) {
    case IK_MODE_BODY:
      doIK = true;
      break;
    case IK_MODE_WORLD:
    case IK_MODE_SUPPORT:
      doIK = true;
      desired = state.xform().inverse() * desired;
      break;
    default:
      doIK = false;
      break;
    }

    bool valid = true;

    if (doIK) {

      Transform3 cur = kbody.manipulatorFK(work,i);
      
      if ( (cur.translation() - desired.translation()).norm() > kbody.DEFAULT_PTOL ||
           quat::dist(cur.rotation(), desired.rotation()) > kbody.DEFAULT_QTOL) {

        if (shouldInitIK[i]) { initIK(i, desired, state.jvalues); }
        
        valid = manipIK(i, desired, state.jvalues, work);

      }
      
    }

    if (!valid) { allOK = false; }
    if (ikvalid) { ikvalid[i] = valid; }

  }

  return allOK;

}

#define debug if (0) std::cerr

bool HuboPlus::comIK( KState& state,
                      const vec3& dcom,
                      const Transform3 manipXforms[NUM_MANIPULATORS],
                      const IKMode mode[NUM_MANIPULATORS],
                      const bool globalIK[NUM_MANIPULATORS],
                      Transform3Array& work,
                      real ascl,
                      real fscl,
                      bool* ikvalid ) const {

  bool ok = false;

  MatX gT, gpT, gxT, fxT, fpT, lambda, gxxpT(6, 3), deltap;
  MatX gfT, deltaf;

  const real alpha = 0.5;
  
  IndexArray pdofs;
  for (size_t i=DOF_POS_X; i<=DOF_ROT_Z; ++i) {
    pdofs.push_back(i);
  }

  IndexArray fdofs;
  for (int i=0; i<4; ++i) {
    if (fscl && mode[i] == IK_MODE_FREE) {
      const IndexArray& jidx = kbody.manipulators[i].jointIndices;
      for (size_t j=0; j<jidx.size(); ++j) {
        fdofs.push_back(jidx[j]);
      }
    }
  }

  for (size_t iter=0; iter<DEFAULT_COM_ITER; ++iter) {

    // try doing IK
    ok = stanceIK( state, manipXforms, mode, globalIK, work, ikvalid );


    // compute the COM pos
    kbody.transforms(state.jvalues, work);

    vec3 com = state.xform() * kbody.com(work);

    // get the error
    vec3 comerr = dcom - com;

    if (comerr.norm() < DEFAULT_COM_PTOL) {
      debug << "breaking after " << iter << " iterations\n";
      break; 
    } else {
      ok = false;
    }

        
    if (ascl == 0) {

      state.body_pos += alpha * comerr;

    } else {
      
      // get jacobians ftw
      kbody.comJacobian(work, pdofs, gpT);
      gpT.block(3, 0, 3, 3) *= ascl;
      

      debug << "gpT=" << gpT << "\n\n";

      if (!fdofs.empty()) {
        kbody.comJacobian(work, fdofs, gfT);
        debug << "gfT=" << gfT << "\n\n";
      }

      gxxpT.setZero();

      for (int i=0; i<4; ++i) {

        if (mode[i] == IK_MODE_WORLD || mode[i] == IK_MODE_SUPPORT) {

          const std::string& name = kbody.manipulators[i].name;

          kbody.comJacobian(work, kbody.manipulators[i].jointIndices, gxT);
          kbody.manipulatorJacobian(work, i, pdofs, fpT);
          kbody.manipulatorJacobian(work, i, fxT);
          lambda = fxT.colPivHouseholderQr().solve(gxT);

          fpT.block(3, 0, 3, 6) *= ascl;

          debug << "gxT[" << name << "]=\n" << gxT << "\n\n";
          debug << "fpT[" << name << "]=\n" << fpT << "\n\n";
          debug << "fxT[" << name << "]=\n" << fxT << "\n\n";
          debug << "lambda[" << name << "]=\n" << lambda << "\n\n";
          gxxpT += fpT * lambda;

        }

      }

      gT = gpT - gxxpT;
      Eigen::Vector3d cerr(comerr[0], comerr[1], comerr[2]);
      deltap = alpha * gT * cerr;

      debug << "gxxpT = \n" << gxxpT << "\n\n";
      debug << "gT = \n" << gT << "\n\n";
      debug << "deltap = \n" << deltap.transpose() << "\n\n";


      vec3 dp(deltap(0), deltap(1), deltap(2));
      vec3 dq(deltap(3), deltap(4), deltap(5));

      state.body_pos += dp;
      state.body_rot = quat::fromOmega(-dq) * state.body_rot;


    }

    if (!fdofs.empty()) {
      Eigen::Vector3d cerr(comerr[0], comerr[1], comerr[2]);
      deltaf = fscl * gfT * cerr;
      debug << "deltaf = \n" << deltaf.transpose() << "\n\n";
      for (size_t i=0; i<fdofs.size(); ++i) {
        state.jvalues[fdofs[i]] += deltaf(i);
      }
    }

  }

  return ok;

}
                           

Transform3 HuboPlus::KState::xform() const {
  return Transform3(body_rot, body_pos);
}




                              
