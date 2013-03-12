#ifndef _HUBOWIDGET_H_
#define _HUBOWIDGET_H_

#include <mzcommon/MzGlWidget.h>
#include <HuboPlus.h>

class HuboWidget: public MzGlWidget {

  Q_OBJECT

public:

  HuboWidget(HuboPlus& hplus, QWidget* parent);
  virtual ~HuboWidget();

public slots:

  void setState(const HuboPlus::KState& state);

protected:

  virtual void initializeGL();
  virtual void paintGL();

private:

  HuboPlus& hplus;
  HuboPlus::KState curState;
  fakerave::Transform3Array xforms;

};

#endif
