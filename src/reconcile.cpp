#include "HuboPlus.h"
#include "HuboKin.h"

using namespace HK;
using namespace fakerave;

size_t findJoint(const KinBody& kbody, IndexArray& jidx, const char* name) {
  size_t ji = kbody.lookupJoint(name);
  jidx.push_back(ji);
  return ji;
}

Transform3 eigen2Xform(const Isometry3d& B) {
  Eigen::Matrix4d me = B.matrix();
  mat4_t<real> mz;
  for (int i=0; i<4; ++i) {
    for (int j=0; j<4; ++j) {
      mz(i,j) = me(i,j);
    }
  }
  Transform3 rval;
  rval.setFromMatrix(mz);
  return rval;
}

int main(int argc, char** argv) {

  std::cout << "loading...\n";
  HuboPlus hplus("../myhubo.kinbody.xml");
  std::cout << "done.\n";

  HuboKin& hkin = hplus.hkin;
  const KinBody& kbody = hplus.kbody;

  RealArray jvalues(kbody.joints.size(), 0.0);
  Transform3Array xforms;
  kbody.transforms(jvalues, xforms);

  const HuboKin::KinConstants& kc = hkin.kc;

  IndexArray jidx;

  size_t LSP = findJoint(kbody, jidx, "LSP");
  size_t HPY = findJoint(kbody, jidx, "HPY");
  size_t LHY = findJoint(kbody, jidx, "LHY");
  size_t LHR = findJoint(kbody, jidx, "LHR");
  size_t LHP = findJoint(kbody, jidx, "LHP");
  size_t LKP = findJoint(kbody, jidx, "LKP");
  size_t LAP = findJoint(kbody, jidx, "LAP");
  size_t LAR = findJoint(kbody, jidx, "LAR");

  Vec3Array jpos(kbody.joints.size());

  for (size_t i=0; i<jidx.size(); ++i) {
    jpos[jidx[i]] = kbody.jointAnchor(xforms, jidx[i]);
    std::cout << "pos[" << kbody.joints[jidx[i]].name << "] = " << jpos[jidx[i]] << "\n";
  }

  vec3 fpos = kbody.manipulatorFK(xforms, 0).translation();
  std::cout << "fpos = " << fpos << "\n\n";

  std::cout << "l1 = " << jpos[LSP].z() - jpos[HPY].z() << "\n";
  std::cout << "kc.leg_l1 = " << kc.leg_l1 << "\n\n";

  std::cout << "l2 = " << jpos[LHY].y() << "\n";
  std::cout << "l2 = " << jpos[LHR].y() << "\n";
  std::cout << "l2 = " << jpos[LAR].y() << "\n";
  std::cout << "kc.leg_l2 = " << kc.leg_l2 << "\n\n";

  std::cout << "l3 = " << jpos[HPY].z() - jpos[LHR].z() << "\n";
  std::cout << "l3 = " << jpos[HPY].z() - jpos[LHP].z() << "\n";
  std::cout << "kc.leg_l3 = " << kc.leg_l3 << "\n\n";

  std::cout << "l4 = " << jpos[LHP].z() - jpos[LKP].z() << "\n";
  std::cout << "l4 = " << jpos[LHR].z() - jpos[LKP].z() << "\n";
  std::cout << "kc.leg_l4 = " << kc.leg_l4 << "\n\n";

  std::cout << "l5 = " << jpos[LKP].z() - jpos[LAP].z() << "\n";
  std::cout << "l5 = " << jpos[LKP].z() - jpos[LAR].z() << "\n";
  std::cout << "kc.leg_l5 = " << kc.leg_l5 << "\n\n";

  std::cout << "l6 = " << jpos[LAP].z() - fpos.z() << "\n";
  std::cout << "l6 = " << jpos[LAR].z() - fpos.z() << "\n";
  std::cout << "kc.leg_l6 = " << kc.leg_l6 << "\n\n";

  size_t lfoot = kbody.lookupBody("Body_LAR");


  real jy = kc.leg_l2;
  real jz = kc.leg_l1 + kc.leg_l3 + kc.leg_l4 + kc.leg_l5 + kc.leg_l6;
  vec3 t0 = xforms[lfoot].transformInv(vec3(0, jy, -jz));

  real s2 = sqrt(2)/2;
  quat rot(0, s2, 0, s2);

  Transform3 relXform(rot, t0);

  std::cout << "relXform = " << relXform << "\n\n";


  // ok, now try running FK thru the dang hkin
  Isometry3d B;
  Vector6d qfk;
  qfk.setZero();

  hkin.legFK(B, qfk, HuboKin::LEFT);
  std::cout << "legFK from zero =\n" << B.matrix() << "\n\n";

  B(0, 3) = 0.04;
  B(2, 3) = -0.85;
  std::cout << "input to legIK =\n" << B.matrix() << "\n\n";

  Vector6d qik;
  hkin.legIK(qik, B, qfk, HuboKin::LEFT);

  std::cout << "qik = " << qik.transpose() << "\n\n";

  hkin.legFK(B, qik, HuboKin::LEFT);
  std::cout << "legFK from qik =\n" << B.matrix() << "\n\n";

  mat2stdvec(qik, jvalues, kbody.manipulators[0].jointIndices);
  kbody.transforms(jvalues, xforms);

  Transform3 BB = kbody.manipulatorFK(xforms, 0);

  std::cout << "my FK from qik =\n" << BB.matrix() << "\n\n";

  std::cout << "my FK from qik (rotation fixed) =\n" << (BB * Transform3(hplus.footRot)).matrix() << "\n\n";
  
}

