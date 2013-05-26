/**
 * \file stl_1.cpp
 * \brief 
 *
 *  \date May 25, 2013
 *  \author arprice
 */

#include "mzcommon/MzGlutApp.h"

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>

#include "zmp/MeshSurface.h"
#include "zmp/stla_io.hpp"

// Decode a bitmasked shared edge
MeshFace::EDGE_INDEX getSharedEdge(unsigned char mask)
{
	// Possible options for an edge are:
	// cba
	// 011 = 3 = AB
	// 110 = 6 = BC
	// 101 = 5 = CA

	switch (mask)
	{
	case 3:
		return MeshFace::EDGE_INDEX::AB;
	case 6:
		return MeshFace::EDGE_INDEX::BC;
	case 5:
		return MeshFace::EDGE_INDEX::CA;
	default:
		return MeshFace::EDGE_INDEX::NONE;
	}
}

void LoadSTLFile(std::string inputFile, SurfaceGraph& graph)
{
	bool error;
	int *face_node;
	double *face_normal;
	int face_num;
	int ierror;
	int node_num;
	double *node_xyz;
	int solid_num;
	int text_num;

	stla_size ( inputFile, &solid_num, &node_num, &face_num, &text_num );

	face_node = new int[3*face_num];
	face_normal = new double[3*face_num];
	node_xyz = new double[3*node_num];

	error = stla_read ( inputFile, node_num, face_num, node_xyz,
			face_node, face_normal );

	if ( error )
	{
		std::cout << "\n";
		std::cout << "  STLA_READ returned error flag.\n";
		return;
	}


	////// Convert to our boost graph

	// Loop through all faces
	for (int faceNum = 0; faceNum < face_num; faceNum++)
	{
		SurfaceGraph::vertex_descriptor v = boost::add_vertex(graph);
		MeshFace& face = graph[v];

		face.faceID = faceNum;

		// Loop through all points
		for (int i = 0; i < 3; i++)
		{
			int nodeID = face_node[i+faceNum*3];
			// Loop through coordinates of points
			for (int j = 0; j < 3; j++)
			{
				face.points[i][j] = node_xyz[j + nodeID * 3];
			}
		}

		// Loop through normal coordinates
		for (int i = 0; i < 3; i++)
		{
			face.plane.normal[i] = face_normal[i+faceNum * 3];
		}

		face.plane.origin = (face.points[0]+face.points[1]+face.points[2])/3;
		face.color[0] = rand()%255;
		face.color[1] = rand()%255;
		face.color[2] = rand()%255;

		//face.color[0] = 128;face.color[1] = 128;face.color[2] = 128;
	}


	// Determine which faces share an edge
	SurfaceGraph::vertex_iterator vertexItA, vertexEndA;

	boost::tie(vertexItA, vertexEndA) = boost::vertices(graph);

	for (; vertexItA != vertexEndA; ++vertexItA)
	{
		SurfaceGraph::vertex_descriptor vertexIDA = *vertexItA; // dereference vertexIt, get the ID
		MeshFace& vertexA = graph[vertexIDA];

		SurfaceGraph::vertex_iterator vertexItB, vertexEndB;
		boost::tie(vertexItB, vertexEndB) = boost::vertices(graph);

		for (; vertexItB != vertexEndB; ++vertexItB)
		{
			SurfaceGraph::vertex_descriptor vertexIDB = *vertexItB; // dereference vertexIt, get the ID
			MeshFace& vertexB = graph[vertexIDB];

			int shareCount = 0;
			unsigned char shared[2];
			// Check if faces share 2 nodes
			for (int i = 0; i < 3; i++)
			{
				for (int j = 0; j < 3; j++)
				{
					float distance = (vertexA.points[i]-vertexB.points[j]).norm();
					if (distance < SMALL_ERROR)
					{
						shared[shareCount] |= 1 << i;
						shared[shareCount] |= 1 << j;
						shareCount++;
					}
				}
			}

			// Found a shared edge
			if (shareCount == 2)
			{
				SurfaceGraph::edge_descriptor e = boost::add_edge(vertexIDA, vertexIDB, graph).first;
				SharedEdge& edge = graph[e];

				edge.faceIdA = vertexA.faceID;
				edge.faceIdB = vertexB.faceID;

				edge.edgeIndexA = getSharedEdge(shared[0]);
				edge.edgeIndexB = getSharedEdge(shared[1]);
			}
		}
	}
}

class STLViewer : public MzGlutApp
{
public:

	SurfaceGraph graph;
	float scale;

	STLViewer(int argc, char** argv) :
		MzGlutApp(argc, argv, GLUT_DOUBLE | GLUT_DEPTH | GLUT_RGB | GLUT_MULTISAMPLE)
	{
		initWindowSize(640, 480);
		createWindow("STL Path Viewer");
		setupBasicLight(vec4f(1, 1, 1, 0));
		double bz = 0.85;

		camera.aim(vec3f(3, 0, bz), vec3f(0, 0, bz), vec3f(0, 0, 1));

		camera.setPerspective();

		camera.setRotateType(GlCamera::ROTATE_2_AXIS);

		camera.setHomePosition();

		setTimer(40, 0);

		scale = 1.0/10.0;
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

		glLineWidth((GLfloat)15.0);

/////// Display code goes here...
		SurfaceGraph::vertex_iterator vertexItA, vertexEndA;
		boost::tie(vertexItA, vertexEndA) = boost::vertices(graph);
		for (; vertexItA != vertexEndA; ++vertexItA)
		{
			SurfaceGraph::vertex_descriptor vertexIDA = *vertexItA; // dereference vertexIt, get the ID
			MeshFace& vertexA = graph[vertexIDA];

			glColor3ub( vertexA.color[0],vertexA.color[1],vertexA.color[2] );
			glBegin(GL_TRIANGLES);
			for (int i = 0; i < 3; i++)
			{
				glVertex3f(vertexA.points[i][0]*scale,vertexA.points[i][1]*scale,vertexA.points[i][2]*scale);
			}
			glEnd( );
		}

		glColor3ub(255,0,0);
		SurfaceGraph::edge_iterator edgeIt, edgeEnd;
		boost::tie(edgeIt, edgeEnd) = boost::edges(graph);
		for (; edgeIt != edgeEnd; ++edgeIt)
		{
			SharedEdge & edge = graph[*edgeIt];

			SurfaceGraph::vertex_descriptor aID = boost::source(*edgeIt, graph);
			SurfaceGraph::vertex_descriptor bID = boost::target(*edgeIt, graph);

			MeshFace& vertexA = graph[aID];
			MeshFace& vertexB = graph[bID];

			glBegin(GL_LINES);
			glVertex3f(vertexA.plane.origin[0]*scale,vertexA.plane.origin[1]*scale,vertexA.plane.origin[2]*scale);
			glVertex3f(vertexB.plane.origin[0]*scale,vertexB.plane.origin[1]*scale,vertexB.plane.origin[2]*scale);
			glEnd();

		}


		glLineWidth((GLfloat)1.0);

		glutSwapBuffers();
	}

};

int main(int argc, char *argv[])
{
	STLViewer viewer(argc, argv);
	//LoadSTLFile("cube.stla", viewer.graph);
	LoadSTLFile("magnolia.stl", viewer.graph);
	viewer.run();
}




