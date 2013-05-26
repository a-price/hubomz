#include <math.h>
#include <vector>
#include <iostream>
#include "footprint.h"
#include "footprint.h"
#include "zmp/draw-footprints.h"
#include <GL/gl.h>
#include <GL/glut.h>

// Thanks too <http://www.dgp.toronto.edu/~ah/csc418/fall_2001/tut/square> for
// showing me how to open an opengl context/window!

std::vector<Footprint> footprints;

void display (void){
  glClear( GL_COLOR_BUFFER_BIT);
  glColor3f(0.0, 1.0, 0.0);
  glBegin(GL_POLYGON);
  drawFootprints(footprints);
  glEnd();
  glFlush();
}

int main(int argc, char **argv) {
  double radius = 3;
  double distance = 13;
  double width = .2;
  double max_step_length = .5;
  double max_step_angle = M_PI / 6;

  Footprint stance_foot = Footprint(0, width, 0, true);

  footprints = walkCircle(radius,
                          distance,
                          width,
                          max_step_length,
                          max_step_angle,
                          stance_foot);

  footprints = turnInPlace(
      2.0,
      width,
      max_step_length,
      stance_foot);

  Footprint end_foot = Footprint(2-width, 2, M_PI/2, true);

  footprints = walkTo(
      width,
      max_step_length,
      max_step_angle,
      stance_foot,
      end_foot);

  glutInit(&argc, argv);
  glutInitDisplayMode ( GLUT_SINGLE | GLUT_RGB | GLUT_DEPTH);

  glutInitWindowPosition(100,100);
  glutInitWindowSize(300,300);
  glutCreateWindow ("square");

  glClearColor(0.0, 0.0, 0.0, 0.0);         // black background
  glMatrixMode(GL_PROJECTION);              // setup viewing projection
  glLoadIdentity();                           // start with identity matrix
  glOrtho(0.0, 10.0, 0.0, 10.0, -1.0, 1.0);   // setup a 10x10x2 viewing world

  glutDisplayFunc(display);
  glutMainLoop();
  return 0;
}
