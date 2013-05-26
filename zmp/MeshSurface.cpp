/**
 * \file MeshSurface.cpp
 * \brief 
 *
 *  \date May 25, 2013
 *  \author arprice
 */

#include "zmp/MeshSurface.h"

INTERSECTION_TYPE inSegment( const Eigen::Vector3f& p, const Eigen::Vector3f& sA, const Eigen::Vector3f& sB, bool checkCollinearity)
{
	Eigen::Vector3f ap = sA - p;
	Eigen::Vector3f bp = sB - p;

	if (checkCollinearity)
	{
		Eigen::Vector3f ab = sA - sB;
		if (ab.cross(ap).norm() > SMALL_ERROR)
		{
			return INTERSECTION_TYPE::NONE;
		}
	}

	ap.normalize();
	bp.normalize();

	return ((ap+bp).norm() < SMALL_ERROR) ? INTERSECTION_TYPE::UNIQUE : INTERSECTION_TYPE::NONE;
}

INTERSECTION_TYPE intersect(const MeshPlane& plane, const Eigen::Vector3f& lineVector, const Eigen::Vector3f& lineOrigin, Eigen::Vector3f& intersection)
{
	float D = plane.normal.dot(lineVector);
	float N = plane.normal.dot(plane.origin-lineOrigin);


	// Check for || line and plane
	// Perpendicular vectors' dot product is 0
	if (D < SMALL_ERROR)
	{
		intersection << (float)NAN,(float)NAN,(float)NAN;
		// Check if plane contains line
		if (N < SMALL_ERROR)
		{
			return INTERSECTION_TYPE::INFINITE;
		}
		else
		{
			return INTERSECTION_TYPE::NONE;
		}
	}

	float sI = N / D;
	intersection = lineOrigin + sI * lineVector;                  // compute segment intersect point

	return INTERSECTION_TYPE::UNIQUE;
}

MeshSurface::MeshSurface()
{
	// TODO Auto-generated constructor stub

}

MeshSurface::~MeshSurface()
{
	// TODO Auto-generated destructor stub
}

bool MeshFace::rayIntersectsEdge(const Ray& ray, EDGE_INDEX edge, Eigen::Vector3f& intersection)
{

}

bool MeshFace::rayIntersectsFace(const Ray& ray, Eigen::Vector3f& intersection)
{

}
