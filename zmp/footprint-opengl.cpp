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

void display (void){
  glClear( GL_COLOR_BUFFER_BIT);
  glColor3f(0.0, 1.0, 0.0);
  glBegin(GL_POLYGON);
  drawFootprints(sampleFootprints(100));
  glEnd();
  glFlush();
}

int main(int argc, char **argv) {
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
