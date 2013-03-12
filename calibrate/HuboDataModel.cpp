#include "HuboDataModel.h"
#include <assert.h>

using namespace fakerave;

HuboDataModel::HuboDataModel(const HuboPlus& h, QObject* parent): 
  QAbstractTableModel(parent), hplus(h) 

{
  
  const char* all_joint_names[] = {

    "RHY", "RHR", "RHP", "RKP", "RAP", "RAR",
    "LHY", "LHR", "LHP", "LKP", "LAP", "LAR",
    "RSP", "RSR", "RSY", "REP", "RWY", "RWP",
    "LSP", "LSR", "LSY", "LEP", "LWY", "LWP",
    "HPY",

    0
  };

#ifdef HAVE_HUBO_ACH
  const size_t hubo_joint_indices[] = { 
    RHY, RHR, RHP, RKN, RAP, RAR,
    LHY, LHR, LHP, LKN, LAP, LAR,
    RSP, RSR, RSY, REB, RWY, RWP,
    LSP, LSR, LSY, LEB, LWY, LWP,
    WST,
  };
#endif

  for (int i=0; all_joint_names[i]; ++i) {
    size_t ji = hplus.kbody.lookupJoint(all_joint_names[i]);
    jnames.push_back(all_joint_names[i]);
    jidx.push_back(ji);
#ifdef HAVE_HUBO_ACH
    hidx.push_back(hubo_joint_indices[i]);
#endif
  }

  cmdState = HuboPlus::KState();

  const KinBody& b = hplus.kbody;

  cmdState.jvalues.resize(b.joints.size(), 0.0);

  cmdState.body_pos = vec3(0,0,1.05);

  curState = cmdState;

  legIKEnabled = false;

  // get the fk for the foot
  size_t manip = hplus.kbody.lookupManipulator("leftFootManip");
  assert( manip != size_t(-1) );

  Transform3Array xforms;
  hplus.kbody.transforms(curState.jvalues, xforms);

  Transform3 lfoot = hplus.kbody.manipulatorFK(xforms, manip);

  defaultFootSep = lfoot.translation().y();
  ikBodyPos = defaultBodyPos = vec3(0,0,1.02);
  ikBodyRot = defaultBodyRot = quat();


#ifdef HAVE_HUBO_ACH

  hubo_state_t state;


  ach_status_t r = achDriver.get(&state);
  assert(HuboAchDriver::useable(r));

  if (HuboAchDriver::useable(r)) {
    for (size_t i=0; i<hidx.size(); ++i) {
      curState.jvalues[jidx[i]] = state.joint[hidx[i]].ref;
      cmdState.jvalues[jidx[i]] = state.joint[hidx[i]].ref;
    }
  }

  r = achDriver.get(&href);
  if (!HuboAchDriver::useable(r)) {
    std::cerr << "warning: couldn't get the first ref!!!!\n";
    for (size_t i=0; i<hidx.size(); ++i) {
      href.ref[hidx[i]] = cmdState.jvalues[jidx[i]];
    }
  }


#endif

}

HuboDataModel::~HuboDataModel() {}

int HuboDataModel::rowCount(const QModelIndex& parent) const {

  if (parent.isValid()) { 
    return 0;
  } else {
    return jidx.size();
  }

}

int HuboDataModel::columnCount(const QModelIndex& parent) const {

  return NUM_COLS;

}


QVariant rad2var(double rad, int role) {

   rad*=180/M_PI;

   if (role == Qt::UserRole) {
     return rad;
   } else {
     return QString::number(rad, 'f', 2);
   }

}

bool HuboDataModel::setData(const QModelIndex& index, 
                            const QVariant& value,
                            int role) {

  if (!index.isValid()) { return false; }

  int col = index.column();

  if (col != COL_CMD_ANGLE) { return false; }

  int row = index.row();

  if (row < 0 || (size_t)row >= jidx.size()) { return false; }

  size_t ji = jidx[row];

  cmdState.jvalues[ji] = value.toDouble() * M_PI/180;

  emit dataChanged( this->index(row, 0),
                    this->index(row, NUM_COLS-1) );

  
  return true;

}


QVariant HuboDataModel::data(const QModelIndex& index, int role) const {

  if (!index.isValid()) { return QVariant(); }

  int col = index.column();

  if (role == Qt::DisplayRole || role == Qt::UserRole) {

    int row = index.row();
    size_t ji = -1;
    
    if (row >= 0 && (size_t)row < jidx.size()) { ji = jidx[row]; }

    switch (col) {
    case COL_CMD_ANGLE:
      if (ji != size_t(-1)) { 
        return rad2var(cmdState.jvalues[ji], role);
      } else {
        return QVariant();
      }
    case COL_CUR_ANGLE:
      if (ji != size_t(-1)) {
        return rad2var(curState.jvalues[ji], role);
      } else { 
        return QVariant();
      }
    case COL_NAME:
      return jnames[row].c_str();
    default:
      return QVariant();
    }

  } else if (role == Qt::TextAlignmentRole && col > 0) {

    return int(Qt::AlignRight | Qt::AlignVCenter);

  } else {

    return QVariant();

  }

    

}

QVariant HuboDataModel::headerData(int section, 
                                   Qt::Orientation orientation, 
                                   int role) const {

  if (role != Qt::DisplayRole) { return QVariant(); }

  if (orientation == Qt::Vertical) { 

    return QVariant();

  } else {

    switch (section) {
    case COL_CMD_ANGLE:
      return "Cmd";
    case COL_CUR_ANGLE:
      return "Cur";
    case COL_NAME:
      return "Joint";
    default:
      return QVariant();
    }

  }

}


Qt::ItemFlags HuboDataModel::flags(const QModelIndex& index) const {
  if (!index.isValid()) { return 0; }
  return Qt::ItemIsEnabled | Qt::ItemIsSelectable;
}

void HuboDataModel::curUpdated(int minRow, int maxRow) {
  emit dataChanged(index(minRow, COL_CUR_ANGLE),
                   index(maxRow, COL_CUR_ANGLE));
}

void HuboDataModel::commandIK(real footSep,
                              const vec3& bodyPos,
                              const quat& bodyRot) {

  Transform3 ctr_from_body(bodyRot, bodyPos);

  size_t manips[2] = { HuboPlus::MANIP_L_FOOT, HuboPlus::MANIP_R_FOOT };
  real sgn[2] = { 1, -1 };

  Transform3Array xforms;
  hplus.kbody.transforms(curState.jvalues, xforms);

  for (int i=0; i<2; ++i) {

    Transform3 ctr_from_foot(vec3(0, sgn[i]*footSep, 0));
    Transform3 body_from_foot = ctr_from_body.inverse() * ctr_from_foot;

    hplus.kbody.manipulatorIK( manips[i], body_from_foot, cmdState.jvalues, xforms );
    
  }

}


void HuboDataModel::updateIK() {
  
  size_t manips[2] = { HuboPlus::MANIP_L_FOOT, HuboPlus::MANIP_R_FOOT };
  
  Transform3 fk[2];
  Transform3Array xforms;

  hplus.kbody.transforms(curState.jvalues, xforms);

  for (int i=0; i<2; ++i) {
    fk[i] = hplus.kbody.manipulatorFK( xforms, manips[i] ).inverse();
  }

  ikBodyPos = 0.5 * (fk[0].translation() + fk[1].translation());
  ikBodyRot = quat::slerp(fk[0].rotation(), fk[1].rotation(), 0.5);



}




void HuboDataModel::handleComms() {

#ifdef HAVE_HUBO_ACH

  hubo_state_t state;

  ach_status_t r = achDriver.get(&href);

  for (size_t i=0; i<hidx.size(); ++i) {
    href.ref[hidx[i]] = cmdState.jvalues[jidx[i]];
  }
  r = achDriver.put(&href);


  r = achDriver.get(&state);
  if (r == ACH_OK) {
    for (size_t i=0; i<hidx.size(); ++i) {
      curState.jvalues[jidx[i]] = state.joint[hidx[i]].ref;
    }
  }

  emit dataChanged(index(0,0), index(hidx.size()-1,NUM_COLS-1)); 
 
  

#else

  int minRow = 1000;
  int maxRow = -1000;
  
  const real maxJointDelta = (360 * M_PI/180) * 0.01;

  const RealArray& cmdvals = cmdState.jvalues;
  RealArray& curvals = curState.jvalues;

  for (size_t i=0; i<jidx.size(); ++i) {

    size_t ji = jidx[i];

    if (ji != size_t(-1) && cmdvals[ji] != curvals[ji]) {

      minRow = std::min(minRow, int(i));
      maxRow = std::max(maxRow, int(i));

      real delta = cmdvals[ji] - curvals[ji];

      if (delta < -maxJointDelta) {
        curvals[ji] -= maxJointDelta;
      } else if (delta > maxJointDelta) {
        curvals[ji] += maxJointDelta;
      } else {
        curvals[ji] = cmdvals[ji];
      }

    }

  }

  if (minRow <= maxRow) {
    curUpdated(minRow, maxRow);
  }
  
#endif

}
