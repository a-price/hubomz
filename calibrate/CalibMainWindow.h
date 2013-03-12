#ifndef _CALIBMAINWINDOW_H_
#define _CALIBMAINWINDOW_H_

#include "ui_CalibMainWindow.h"
#include "fakerave.h"

class HuboPlus;
class HuboDataModel;
class HuboWidget;
class QDoubleSpinBox;

class CalibMainWindow: public QMainWindow, private Ui_CalibMainWindow {

  Q_OBJECT

public:

  CalibMainWindow(HuboPlus& hplus, QWidget* parent=0);
  virtual ~CalibMainWindow();

  fakerave::vec3 getBodyPos();
  void setBodyPos(const fakerave::vec3&);

  fakerave::quat getBodyRot();
  void setBodyRot(const fakerave::quat&);

public slots:
  
  void onCurrentChanged(const QModelIndex& current,
                        const QModelIndex& previous);

  void onCmdClicked();

  void onRelClicked();

  void onHomeAllClicked();

  void onModelUpdated();
  
  void onTimer();

  void onLegIKToggled();

  void onLegIKChanged();

  void onLegIKReset();

private:

  HuboDataModel* model;
  HuboWidget* hdisplay;

  QDoubleSpinBox* bodyPosSpin[3];
  QDoubleSpinBox* bodyRotSpin[3];

};

#endif
