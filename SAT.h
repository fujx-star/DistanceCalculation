#pragma once
#include "OBB.h"
#include "Common.h"
#include <array>

struct ProjectionParam
{
	Real value;
	Point point;
};

struct ProjectionInterval
{
	ProjectionParam start;
	ProjectionParam end;
};

bool GT(Real a, Real b)
{
	return a > b + TOL;
}

bool LT(Real a, Real b)
{
	return a < b + TOL;
}

bool EQ(Real a, Real b)
{
	return fabs(a - b) < TOL;
}

bool NE(Real a, Real b)
{
	return fabs(a - b) >= TOL;
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
	std::sort(params.begin(), params.end(), cmp);
}

// return negative value if no overlap, else return the length of overlap
Real overlap(ProjectionInterval&& a, ProjectionInterval&& b)
{
	Real overlapStart = std::max(a.start.value, b.start.value);
	Real overlapEnd = std::min(a.end.value, b.end.value);

	return overlapEnd - overlapStart;
}

Real minDistSAT(const OBB& a, const OBB& b, std::pair<Point, Point>& pointPair) {
	Real sqrDist{ 1000 };

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
		glm::normalize(glm::cross(a.u[0], b.u[0])),
		glm::normalize(glm::cross(a.u[0], b.u[1])),
		glm::normalize(glm::cross(a.u[0], b.u[2])),
		glm::normalize(glm::cross(a.u[1], b.u[0])),
		glm::normalize(glm::cross(a.u[1], b.u[1])),
		glm::normalize(glm::cross(a.u[1], b.u[2])),
		glm::normalize(glm::cross(a.u[2], b.u[0])),
		glm::normalize(glm::cross(a.u[2], b.u[1])),
		glm::normalize(glm::cross(a.u[2], b.u[2]))
	};

	std::array<std::array<ProjectionParam, 8>, 15> aProjRes, bProjRes;
	Real overlapIntervalLen{ 0 };
	int axisInd{ -1 };
	for (int i = 0; i < 15; i++)
	{
		project(aPoints, axes[i], aProjRes[i]);
		project(bPoints, axes[i], bProjRes[i]);
		auto aProj = aProjRes[i];
		auto bProj = bProjRes[i];
		Real tmp = overlap(
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
		//std::array<ProjectionParam, 8> aProj = aProjRes[axisInd];
		//std::array<ProjectionParam, 8> bProj = bProjRes[axisInd];
		//ProjectionInterval aInt = { aProj[0], aProj[7] };
		//ProjectionInterval bInt = { bProj[0], bProj[7] };

		//std::vector<Point> aEqualPoints, bEqualPoints;
		//if (aInt.start.value >= bInt.end.value)
		//{
		//	for (int i = 0; i < 8; i++)
		//	{
		//		if (EQ(aProjRes[axisInd][i].value, aInt.start.value))
		//		{
		//			aEqualPoints.push_back(aProjRes[axisInd][i].point);
		//		}
		//		if (EQ(bProjRes[axisInd][i].value, bInt.end.value))
		//		{
		//			bEqualPoints.push_back(bProjRes[axisInd][i].point);
		//		}
		//	}
		//}
		//else
		//{
		//	for (int i = 0; i < 8; i++)
		//	{
		//		if (EQ(aProjRes[axisInd][i].value, aInt.end.value))
		//		{
		//			aEqualPoints.push_back(aProjRes[axisInd][i].point);
		//		}
		//		if (EQ(bProjRes[axisInd][i].value, bInt.start.value))
		//		{
		//			bEqualPoints.push_back(bProjRes[axisInd][i].point);
		//		}
		//	}
		//}
		//pointPair = { aEqualPoints[0], bEqualPoints[0] };
		//// if point-point or point-edge, calcu min distance and point pair
		//int aPointType = aEqualPoints.size();
		//int bPointType = bEqualPoints.size();
		//if (aPointType == 1)
		//{
		//	if (bPointType == 1)
		//	{
		//		Real distSqr = glm::dot(pointPair.second - pointPair.first, pointPair.second - pointPair.first);
		//		return distSqr;
		//	}
		//	else if (bPointType == 2)
		//	{
		//		Real distSqr = distancePointSegment(aEqualPoints[0], bEqualPoints[0], bEqualPoints[1], pointPair);
		//		return distSqr;
		//	}
		//	else
		//	{
		//		Real distSqr = distancePointRect(aEqualPoints[0], bEqualPoints[0], bEqualPoints[1], bEqualPoints[2], bEqualPoints[3], pointPair);
		//		return distSqr;
		//	}
		//}
		//else if (aPointType == 2)
		//{
		//	if (bPointType == 1)
		//	{
		//		Real distSqr = distancePointSegment(bEqualPoints[0], aEqualPoints[0], aEqualPoints[1], pointPair);
		//		std::swap(pointPair.first, pointPair.second);
		//		return distSqr;
		//	}
		//	else if (bPointType == 2)
		//	{
		//		Real distSqr = distanceSegmentSegment(aEqualPoints[0], aEqualPoints[1], bEqualPoints[0], bEqualPoints[1], pointPair);
		//		return distSqr;
		//	}
		//	else
		//	{
		//		// min dist between segment and rect
		//		Real distSqr = distanceSegmentRect(aEqualPoints[0], aEqualPoints[1], bEqualPoints[0], bEqualPoints[1], bEqualPoints[2], bEqualPoints[3], pointPair);
		//		return distSqr;
		//	}
		//}
		//else
		//{
		//	if (bPointType == 1)
		//	{
		//		Real distSqr = distancePointRect(bEqualPoints[0], aEqualPoints[0], aEqualPoints[1], aEqualPoints[2], aEqualPoints[3], pointPair);
		//		return distSqr;
		//	}
		//	else if (bPointType == 2)
		//	{
		//		// min dist between segment and rect
		//		Real distSqr = distanceSegmentRect(bEqualPoints[0], bEqualPoints[1], aEqualPoints[0], aEqualPoints[1], aEqualPoints[2], aEqualPoints[3], pointPair);
		//		std::swap(pointPair.first, pointPair.second);
		//		return distSqr;
		//	}
		//	else
		//	{
		//		// min dist between rects
		//		return overlapIntervalLen * overlapIntervalLen;
		//	}
		//}
		return overlapIntervalLen * overlapIntervalLen;
	}
	else
	{
		return 0;
	}
}

// if a and b contains each other, then the result is not valid
Real seperate(ProjectionInterval&& a, ProjectionInterval&& b)
{
	bool valid1 = a.start.value < b.start.value;
	bool valid2 = a.end.value < b.end.value;
	if (valid1 ^ valid2)
	{
		return -1;
	}

	Real seperateStart = std::min(a.start.value, b.start.value);
	Real seperateEnd = std::max(a.end.value, b.end.value);

	return seperateEnd - seperateStart;
}


// only consider the case that a and b are not intersected
Real maxDistSAT(const OBB& a, const OBB& b, std::pair<Point, Point>& pointPair) {
	Real sqrDist{ 0.0 };

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
	Real seperateIntervalLen{ 0 };
	Real overlapIntervalLen{ 0 };
	int axisInd{ -1 };
	for (int i = 0; i < 15; i++)
	{
		project(aPoints, axes[i], aProjRes[i]);
		project(bPoints, axes[i], bProjRes[i]);
		auto aProj = aProjRes[i];
		auto bProj = bProjRes[i];
		Real tmp = seperate(
			ProjectionInterval{ aProj[0], aProj[7] },
			ProjectionInterval{ bProj[0], bProj[7] }
		);
#ifdef DEBUG_DISTANCE
		std::cout << "SA: (" << axes[i].x << ", " << axes[i].y << ", " << axes[i].z << ") --- aProj: (" << aProj[0].value << ", " << aProj[7].value << "), bProj: (" << bProj[0].value << ", " << bProj[7].value << "), overlap: " << tmp << std::endl;
#endif
		if (tmp > seperateIntervalLen)
		{
			seperateIntervalLen = tmp;
			axisInd = i;
		}
		tmp = overlap(
			ProjectionInterval{ aProj[0], aProj[7] },
			ProjectionInterval{ bProj[0], bProj[7] }
		);
		if (tmp < overlapIntervalLen)
		{
			overlapIntervalLen = tmp;
		}
	}

	// 如果两OBB分离才执行下面的代码
	if (overlapIntervalLen > 0)
	{
		return -1;
	}
	std::array<ProjectionParam, 8> aProj = aProjRes[axisInd];
	std::array<ProjectionParam, 8> bProj = bProjRes[axisInd];
	ProjectionInterval aInt = { aProj[0], aProj[7] };
	ProjectionInterval bInt = { bProj[0], bProj[7] };

	std::array<Point, 4> aEqualPoints, bEqualPoints;
	int aPointNum{ 0 }, bPointNum{ 0 };
	if (aInt.start.value < bInt.start.value)	// [aStart, bEnd]
	{
		for (int i = 0; i < 8; i++)
		{
			if (EQ(aProjRes[axisInd][i].value, aInt.start.value))
			{
				aEqualPoints[aPointNum++] = aProjRes[axisInd][i].point;
			}
			if (EQ(bProjRes[axisInd][i].value, bInt.end.value))
			{
				bEqualPoints[bPointNum++] = bProjRes[axisInd][i].point;
			}
		}
	}
	else	// [bStart, aEnd]
	{
		for (int i = 0; i < 8; i++)
		{
			if (EQ(aProjRes[axisInd][i].value, aInt.end.value))
			{
				aEqualPoints[aPointNum++] = aProjRes[axisInd][i].point;
			}
			if (EQ(bProjRes[axisInd][i].value, bInt.start.value))
			{
				bEqualPoints[bPointNum++] = bProjRes[axisInd][i].point;
			}
		}
	}
	Real maxDistSqr{ 0 };
	for (int i = 0; i < aPointNum; i++)
	{
		for (int j = 0; j < bPointNum; j++)
		{
			Real sqrD = glm::distance2(aEqualPoints[i], bEqualPoints[j]);
			if (sqrD > maxDistSqr)
			{
				maxDistSqr = sqrD;
				pointPair = { aEqualPoints[i], bEqualPoints[j] };
			}
		}
	}
	return sqrt(maxDistSqr);
}