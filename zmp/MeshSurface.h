/**
 * \file MeshSurface.h
 * \brief 
 *
 *  \date May 25, 2013
 *  \author arprice
 */

#ifndef MESHSURFACE_H_
#define MESHSURFACE_H_

#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <math.h>
#include <limits>

#include <boost/graph/adjacency_list.hpp>


const float SMALL_ERROR = 0.000001;

enum INTERSECTION_TYPE
{
	NONE,
	UNIQUE,
	MULTIPLE,
	INFINITE
};

class Ray
{
public:
	Eigen::Vector3f origin;
	Eigen::Vector3f vector;
};

class MeshPlane
{
public:
	Eigen::Vector3f normal;
	float distance;

	Eigen::Vector3f origin;
};

// inSegment(): determine if a point is inside a segment
//    Input:  a point P, and a collinear segment S from point A to point B
//    Return: true = P is inside S
//            false = P is  not inside S
INTERSECTION_TYPE inSegment( const Eigen::Vector3f& p, const Eigen::Vector3f& sA, const Eigen::Vector3f& sB, bool checkCollinearity = false);

INTERSECTION_TYPE intersect(const MeshPlane& plane, const Eigen::Vector3f& lineVector, const Eigen::Vector3f& lineOrigin, Eigen::Vector3f& intersection);



class MeshFace
{
public:
	enum EDGE_INDEX : signed int
	{
		AB=0,
		BC=1,
		CA=2,
		NONE=-1
	};

	unsigned faceID;
	unsigned char color[3];
	Eigen::Vector3f points[3];
	MeshPlane plane;


	bool rayIntersectsEdge(const Ray& ray, EDGE_INDEX edge, Eigen::Vector3f& intersection);
	bool rayIntersectsFace(const Ray& ray, Eigen::Vector3f& intersection);
};

class SharedEdge
{
public:
	unsigned faceIdA;
	unsigned faceIdB;
	MeshFace::EDGE_INDEX edgeIndexA;
	MeshFace::EDGE_INDEX edgeIndexB;
};

class MeshSurface
{
public:
	MeshSurface();
	virtual ~MeshSurface();
};

typedef boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS, MeshFace, SharedEdge> SurfaceGraph;

#endif /* MESHSURFACE_H_ */
