#pragma once
#include "OBB.h"
#include "Common.h"
#include <array>
#define TOL 1e-6

struct ProjectionParam
{
	float value;
	Point point;
};

struct ProjectionInterval
{
	ProjectionParam start;
	ProjectionParam end;
};

bool GT(float a, float b)
{
	return a > b + TOL;
}

bool LT(float a, float b)
{
	return a < b + TOL;
}

bool EQ(float a, float b)
{
	return fabs(a - b) < TOL;
}

void project(const std::array<Point, 8>& points, const Vector& axis, std::array<ProjectionParam, 8>& params)
{
	for (int i = 0; i < 8; i++)
	{
		params[i].point = points[i];
		params[i].value = glm::dot(points[i], axis);
	}
	auto cmp = [](const ProjectionParam& p1, const ProjectionParam& p2) -> bool {
		return p1.value < p2.value;
		};
	sort(params.begin(), params.end(), cmp);
}

float overlap(ProjectionInterval&& a, ProjectionInterval&& b)
{
	float overlapStart = std::max(a.start.value, b.start.value);
	float overlapEnd = std::min(a.end.value, b.end.value);

	return overlapEnd - overlapStart;
}

float distanceSAT(const OBB& a, const OBB& b, std::pair<Point, Point>& pointPair) {
	float sqrDist{ 1000 };

	std::array<Point, 8> aPoints = {
		a.getPoint(0),
		a.getPoint(1),
		a.getPoint(2),
		a.getPoint(3),
		a.getPoint(4),
		a.getPoint(5),
		a.getPoint(6),
		a.getPoint(7)
	};

	std::array<Point, 8> bPoints = {
		b.getPoint(0),
		b.getPoint(1),
		b.getPoint(2),
		b.getPoint(3),
		b.getPoint(4),
		b.getPoint(5),
		b.getPoint(6),
		b.getPoint(7)
	};

	std::array<Vector, 15> axes = {
		a.u[0],
		a.u[1],
		a.u[2],
		b.u[0],
		b.u[1],
		b.u[2],
		glm::cross(a.u[0], b.u[0]),
		glm::cross(a.u[0], b.u[1]),
		glm::cross(a.u[0], b.u[2]),
		glm::cross(a.u[1], b.u[0]),
		glm::cross(a.u[1], b.u[1]),
		glm::cross(a.u[1], b.u[2]),
		glm::cross(a.u[2], b.u[0]),
		glm::cross(a.u[2], b.u[1]),
		glm::cross(a.u[2], b.u[2])
	};

	std::array<std::array<ProjectionParam, 8>, 15> aProjRes, bProjRes;
	float overlapIntervalLen{ 0 };
	int axisInd{ -1 };
	for (int i = 0; i < 15; i++)
	{
		project(aPoints, axes[i], aProjRes[i]);
		project(bPoints, axes[i], bProjRes[i]);
		auto aProj = aProjRes[i];
		auto bProj = bProjRes[i];
		float tmp = overlap(
			ProjectionInterval{ aProj[0], aProj[7] }, 
			ProjectionInterval{ bProj[0], bProj[7] }
		);
#ifdef DEBUG_DISTANCE
		std::cout << "SA: (" << axes[i].x << ", " << axes[i].y << ", " << axes[i].z << ") --- aProj: (" << aProj[0].value << ", " << aProj[7].value << "), bProj: (" << bProj[0].value << ", " << bProj[7].value << "), overlap: " << tmp << std::endl;
#endif
		if (tmp < overlapIntervalLen)
		{
			overlapIntervalLen = tmp;
			axisInd = i;
		}
	}


	if (axisInd >= 0)
	{
		std::array<ProjectionParam, 8> aProj = aProjRes[axisInd];
		std::array<ProjectionParam, 8> bProj = bProjRes[axisInd];
		ProjectionInterval aInt = { aProj[0], aProj[7] };
		ProjectionInterval bInt = { bProj[0], bProj[7] };

		std::vector<Point> aEqualPoints, bEqualPoints;
		if (aInt.start.value >= bInt.end.value)
		{
			for (int i = 0; i < 8; i++)
			{
				if (EQ(aProjRes[axisInd][i].value, aInt.start.value))
				{
					aEqualPoints.push_back(aProjRes[axisInd][i].point);
				}
				if (EQ(bProjRes[axisInd][i].value, bInt.end.value))
				{
					bEqualPoints.push_back(bProjRes[axisInd][i].point);
				}
			}
		}
		else
		{
			for (int i = 0; i < 8; i++)
			{
				if (EQ(aProjRes[axisInd][i].value, aInt.end.value))
				{
					aEqualPoints.push_back(aProjRes[axisInd][i].point);
				}
				if (EQ(bProjRes[axisInd][i].value, bInt.start.value))
				{
					bEqualPoints.push_back(bProjRes[axisInd][i].point);
				}
			}
		}
		pointPair = { aEqualPoints[0], bEqualPoints[0] };
		// if point-point or point-edge, calcu min distance and point pair
		int aPointType = aEqualPoints.size();
		int bPointType = bEqualPoints.size();
		if (aPointType + bPointType < 4)
		{
			if (aPointType == 1 && bPointType == 2)
			{
				float distSqr = distancePointSegment(aEqualPoints[0], bEqualPoints[0], bEqualPoints[1], pointPair);
				return distSqr;
			}
			else if (aPointType == 2 && bPointType == 1)
			{
				float distSqr = distancePointSegment(aEqualPoints[0], bEqualPoints[0], bEqualPoints[1], pointPair);
				std::swap(pointPair.first, pointPair.second);
				return distSqr;
			}
			else if (aPointType == 2 && bPointType == 2)
			{
				float distSqr = distanceSegmentSegment(aEqualPoints[0], aEqualPoints[1], bEqualPoints[0], bEqualPoints[1], pointPair);
				return distSqr;
			}
		}
		// else, calcu point pair(not support)
		else {
			// point-face
			// edge-face: according to parallel, degenerate to edge-edge and point-face
			// face-face: according to parallel, degenerate to point-face
			return overlapIntervalLen * overlapIntervalLen;
		}
	}
	else
	{
		return 0;
	}
}