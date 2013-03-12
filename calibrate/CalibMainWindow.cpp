#include "CalibMainWindow.h"
#include "HuboDataModel.h"
#include "HuboWidget.h"
#include <QLayout>
#include <QTimer>


using namespace fakerave;

const int timerIntervalMsec = 10;
const real timerDT = timerIntervalMsec / 1000.0;



CalibMainWindow::CalibMainWindow(HuboPlus& hplus, QWidget* parent) {

  setupUi(this);

  model = new HuboDataModel(hplus, this);

  bodyPosSpin[0] = bodyXSpin;
  bodyPosSpin[1] = bodyYSpin;
  bodyPosSpin[2] = bodyZSpin;

  bodyRotSpin[0] = bodyRollSpin;
  bodyRotSpin[1] = bodyPitchSpin;
  bodyRotSpin[2] = bodyYawSpin;

  onLegIKReset();

  jointTableView->setModel(model);

  jointTableView->resizeColumnsToContents();

  jointTableView->horizontalHeader()->setStretchLastSection(true);
  jointTableView->verticalHeader()->hide();

  connect(jointTableView->selectionModel(),
          SIGNAL(currentChanged(const QModelIndex&, const QModelIndex&)),
          this, SLOT(onCurrentChanged(const QModelIndex&, const QModelIndex&)));

  connect(cmdGoButton, SIGNAL(clicked()),
          this, SLOT(onCmdClicked()));

  connect(relGoButton, SIGNAL(clicked()),
          this, SLOT(onRelClicked()));

  connect(homeAllButton, SIGNAL(clicked()),
          this, SLOT(onHomeAllClicked()));


  homeAllButton->setEnabled(false);
  zeroButton->setEnabled(false);

  jointTableView->setCurrentIndex(model->index(0,0));

  QVBoxLayout* vl = new QVBoxLayout(glContainer);
  vl->setMargin(0);

  hdisplay = new HuboWidget(hplus, glContainer);

  vl->addWidget(hdisplay);

  connect(model, SIGNAL(dataChanged(const QModelIndex&, const QModelIndex&)),
          this, SLOT(onModelUpdated()));

  connect(legIKEnableButton, SIGNAL(clicked()),
          this, SLOT(onLegIKToggled()));

  connect(legIKDisableButton, SIGNAL(clicked()),
          this, SLOT(onLegIKToggled()));

  connect(legIKResetButton, SIGNAL(clicked()),
          this, SLOT(onLegIKReset()));

  for (int i=0; i<3; ++i) {
    connect(bodyPosSpin[i], SIGNAL(valueChanged(double)),
            this, SLOT(onLegIKChanged()));
    connect(bodyRotSpin[i], SIGNAL(valueChanged(double)),
            this, SLOT(onLegIKChanged()));
  }

  hdisplay->setState(model->curState);

  QTimer* timer = new QTimer(this);

  connect(timer, SIGNAL(timeout()), this, SLOT(onTimer()));

  timer->start(timerIntervalMsec);

}

CalibMainWindow::~CalibMainWindow() {
}

void CalibMainWindow::onModelUpdated() {

  vec3 p;
  quat q;

  if (model->legIKEnabled) {

    model->updateIK();
    p = model->ikBodyPos;
    q = model->ikBodyRot;
    
  } else {

    p = model->cmdState.body_pos;
    q = model->cmdState.body_rot;

  }

  model->curState.body_pos = p;
  model->curState.body_rot = q;

  hdisplay->setState(model->curState);

}

void CalibMainWindow::onLegIKReset() {

  footSepSpin->setValue(model->defaultFootSep);

  setBodyPos(model->defaultBodyPos);
  setBodyRot(model->defaultBodyRot);

}

void CalibMainWindow::onLegIKToggled() {

  bool& e = model->legIKEnabled;

  legIKEnableButton->setEnabled(e);
  legIKResetButton->setEnabled(e);
  footSepSpin->setEnabled(e);
  footSepLabel->setEnabled(e);
  legIKDisableButton->setEnabled(!e);

  e = !e;

  onLegIKChanged();
  onModelUpdated();

}

void CalibMainWindow::onLegIKChanged() {
  if (model->legIKEnabled) {
    model->commandIK(footSepSpin->value(),
                     getBodyPos(), getBodyRot());
  }
}

void CalibMainWindow::onCurrentChanged(const QModelIndex& current,
                                       const QModelIndex& previous) {


  jointGroupBox->setEnabled(current.isValid());
  
  if (current.isValid()) {
    QString name = "Joint " + model->data(current.sibling(current.row(), 0)).toString();
    jointGroupBox->setTitle(name);

    bool wasBlocking = cmdSpin->blockSignals(true);

    QVariant cur = model->data(current.sibling(current.row(), HuboDataModel::COL_CMD_ANGLE), Qt::UserRole);
    
    cmdSpin->setValue(cur.toDouble());

    cmdSpin->blockSignals(wasBlocking);

  }

}

void CalibMainWindow::onCmdClicked() {

  QModelIndex current = jointTableView->currentIndex();
  if (!current.isValid()) { return; }

  QModelIndex cmd = current.sibling(current.row(), HuboDataModel::COL_CMD_ANGLE);
 
  model->setData(cmd, cmdSpin->value());

}

void CalibMainWindow::onRelClicked() {

  cmdSpin->setValue(cmdSpin->value() + relSpin->value());
  onCmdClicked();

}

void CalibMainWindow::onHomeAllClicked() {
  // nop
}

void CalibMainWindow::onTimer() {

  model->handleComms();

}

static inline vec3 getVec(QDoubleSpinBox* spin[3]) {
  vec3 rval;
  for (int i=0; i<3; ++i) { rval[i] = spin[i]->value(); }
  return rval;
}

static inline void setVec(QDoubleSpinBox* spin[3], const vec3& v) {
  for (int i=0; i<3; ++i) {
    bool wasBlocked = spin[i]->blockSignals(true);
    spin[i]->setValue(v[i]);
    spin[i]->blockSignals(wasBlocked);
  }
}


vec3 CalibMainWindow::getBodyPos() {
  return getVec(bodyPosSpin);
}

void CalibMainWindow::setBodyPos(const vec3& v) {
  setVec(bodyPosSpin, v);
}

quat CalibMainWindow::getBodyRot() {
  return quat::fromEuler(getVec(bodyRotSpin)*M_PI/180);
}

void CalibMainWindow::setBodyRot(const quat& q) {
  setVec(bodyRotSpin, q.toEuler()*180/M_PI);
}

