#include "mzcommon/MzGlutApp.h"

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

class SpaceVector
{
public:
	Eigen::Vector3d basePos;
	Eigen::Vector3d relTargetPos;
	Eigen::Vector3d absTargetPos()
	{
		return basePos + relTargetPos;
	}
};

class VectorViewer : public MzGlutApp
{
public:

	std::vector<SpaceVector> SpaceVectors;

	VectorViewer(int argc, char** argv) : 
		MzGlutApp(argc, argv, GLUT_DOUBLE | GLUT_DEPTH | GLUT_RGB | GLUT_MULTISAMPLE)
	{
		initWindowSize(640, 480);
		createWindow("Vector Viewer");
		setupBasicLight(vec4f(1, 1, 1, 0));
		double bz = 0.85;

		camera.aim(vec3f(3, 0, bz), vec3f(0, 0, bz), vec3f(0, 0, 1));

		camera.setPerspective();

		camera.setRotateType(GlCamera::ROTATE_2_AXIS);

		camera.setHomePosition();

		setTimer(40, 0);
	}

	virtual void display()
	{

		MzGlutApp::display();

		glMatrixMode(GL_MODELVIEW);

		// Draw Grid
		glColor3ub(0, 127, 127);
		glNormal3f(0, 0, 1);
		glBegin(GL_LINES);
		double s = 0.5;
		double n = 10;
		double x = s * n;
		for (int i = -n; i <= n; ++i)
		{
			glVertex2d(x, i * s);
			glVertex2d(-x, i * s);
			glVertex2d(i * s, x);
			glVertex2d(i * s, -x);
		}
		glEnd();
		
		// Draw Vectors
		
		glLineWidth((GLfloat)9.0);
		glBegin(GL_LINES);
		for (int i = 0; i < (int)SpaceVectors.size(); i++)
		{
		//std::cout << SpaceVectors[i].relTargetPos.matrix().transpose() << std::endl;
			double n = SpaceVectors[i].relTargetPos.norm();
			glColor3d(fabs(SpaceVectors[i].relTargetPos.x())/n,
					  fabs(SpaceVectors[i].relTargetPos.y())/n,
					  fabs(SpaceVectors[i].relTargetPos.z())/n);
					  
			glVertex3d(SpaceVectors[i].basePos.x(),
			           SpaceVectors[i].basePos.y(),
			           SpaceVectors[i].basePos.z());
			
			glVertex3d(SpaceVectors[i].absTargetPos().x(),
			           SpaceVectors[i].absTargetPos().y(),
			           SpaceVectors[i].absTargetPos().z());
		}
		glEnd( );
		glLineWidth((GLfloat)1.0);
		
		glutSwapBuffers();
	}

};

void ReadVectors(VectorViewer& viewer, std::string filename = "vectors.txt")
{
	std::ifstream file ( filename.c_str() );
	std::string line;
	std::string value;
	std::stringstream lineStream;
	
	Eigen::Vector3d base;
	Eigen::Vector3d target;
	double val = 0;
	while ( file.good() )
	{
		//Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
		SpaceVector sv;
		std::getline ( file, line );
		//std::cout << line << ": ";
		lineStream << line;
		
		for (int i = 0; i < 6; i++)
		{
			std::getline ( lineStream, value, ' ' );
			
			//std::cout << value << "\t";
			val = atof(value.c_str());
			switch (i)
			{
			case 0: base.x() = val; break;
			case 1: base.y() = val; break;
			case 2: base.z() = val; break;
			case 3: target.x() = val; break;
			case 4: target.y() = val; break;
			case 5: target.z() = val; break;
			}
		}
		//std::getline ( file, value, '\n' );
		//target.z() = val;
		
		sv.basePos = base;
		sv.relTargetPos = target;
		
		//std::cout << pose.matrix() << std::endl;
		viewer.SpaceVectors.push_back(sv);
		
		lineStream.clear();
		//std::cout << std::endl;
	}
}

void SampleVectors(VectorViewer& viewer)
{
	int numVecs = 10;
	
}

int main(int argc, char** argv)
{
	VectorViewer viewer(argc, argv);
	ReadVectors(viewer, "vectors.txt");
	viewer.run();
}

