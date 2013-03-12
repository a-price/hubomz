#include <QApplication>
#include "CalibMainWindow.h"
#include "HuboPlus.h"

int main(int argc, char** argv) {
  
  QApplication app(argc, argv);

  HuboPlus hplus("../hubo/myhubo.kinbody.xml");


  app.connect(&app, SIGNAL(lastWindowClosed()),
              &app, SLOT(quit()));

  CalibMainWindow w(hplus);

  w.show();

  return app.exec();

}
