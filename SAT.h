#pragma once
#include "OBB.h"
#include "Common.h"
#include <array>

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

ProjectionInterval project(const std::array<Point, 8>& points, const Vector& axis)
{
	float val = glm::dot(points[0], axis);
	ProjectionParam min{ val, points[0] };
	ProjectionParam max{ val, points[0] };
	for (unsigned int i = 1; i < 8; i++)
	{
		Point p = points[i];
		float proj = glm::dot(p, axis);
		if (proj > max.value)
		{
			max.value = proj;
			max.point = p;
		}
		if (proj < min.value)
		{
			min.value = proj;
			min.point = p;
		}
	}
	return { min, max };
}

float overlap(ProjectionInterval& a, ProjectionInterval& b)
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

	float overlapIntervalLen{ 0 };

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
	for (const auto& axis : axes)
	{
		ProjectionInterval aProj = project(aPoints, axis);
		ProjectionInterval bProj = project(bPoints, axis);
		float tmp = overlap(aProj, bProj);
#ifdef DEBUG_DISTANCE
		std::cout << "SA: (" << axis.x << ", " << axis.y << ", " << axis.z << ") --- aProj: (" << aProj.start.value << ", " << aProj.end.value << "), bProj: (" << bProj.start.value << ", " << bProj.end.value << "), overlap: " << tmp << std::endl;
#endif
		if (tmp < overlapIntervalLen)
		{
			overlapIntervalLen = tmp;
			if (aProj.start.value >= bProj.end.value)
			{
				pointPair = { bProj.end.point, aProj.start.point };
			}
			else
			{
				pointPair = { aProj.end.point, bProj.start.point };
			}
		}
	}

	if (overlapIntervalLen < 0)
	{
		Vector diff = pointPair.second - pointPair.first;
		return glm::dot(diff, diff);
	}
	else
	{
		return 0;
	}
}