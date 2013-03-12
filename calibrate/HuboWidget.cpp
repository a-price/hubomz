#include "HuboWidget.h"

HuboWidget::HuboWidget(HuboPlus& h, QWidget* parent): 
  MzGlWidget(parent), hplus(h) 
 
{

  double bz = 0.85;

  GlCamera& camera = this->camera();

  camera.aim(vec3f(3, 0, bz),
             vec3f(0, 0, bz),
             vec3f(0, 0, 1));

  camera.setPerspective();
  
  camera.setRotateType(GlCamera::ROTATE_2_AXIS);
  
  camera.setHomePosition();

  
  curState.jvalues.resize(hplus.kbody.joints.size(), 0.0);
  
  setState(curState);

}

void HuboWidget::setState(const HuboPlus::KState& state) { 

  curState = state;
  hplus.kbody.transforms(curState.jvalues, xforms);
  updateGL();

}

HuboWidget::~HuboWidget() {}


void HuboWidget::initializeGL() {
  MzGlWidget::initializeGL();
  hplus.kbody.compileDisplayLists();
  MzGlWidget::paintGL();
  setupBasicLight(vec4f(1, -1, 1, 0));
}

void HuboWidget::paintGL() {

  MzGlWidget::paintGL();

  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();

  glstuff::mult_transform(curState.xform());

  hplus.render(xforms);

  glPopMatrix();

  glColor3ub(0, 0, 127);
  glNormal3f(0, 0, 1);
  glBegin(GL_LINES);
  double s = 0.5;
  double n = 10;
  double x = s*n;
  for (int i=-n; i<=n; ++i) {
    glVertex2d(x, i*s);
    glVertex2d(-x, i*s);
    glVertex2d(i*s, x);
    glVertex2d(i*s, -x);
  }
  glEnd();



}
