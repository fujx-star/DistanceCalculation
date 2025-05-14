#pragma once
#include "OBB.h"
#include "Common.h"
#include <array>
#include <mutex>
#include <omp.h>

constexpr Real interval = 0.01;

Point sam(const Point& o, const Point& oa, const Point& ob, Real u, Real v)
{
	return o + oa * u + ob * v;
}

Real minSample(const Point& o1, const Point& a1, const Point& b1, const Point& o2, const Point& a2, const Point& b2, std::pair<Point, Point>& pointPair)
{
	Real minDistSqr{ 1000000 };

	Vector oa1 = a1 - o1;
	Vector ob1 = b1 - o1;
	Vector oa2 = a2 - o2;
	Vector ob2 = b2 - o2;

	for (Real a = 0.0; a < 1.0; a += interval)
	{
		for (Real b = 0.0; b < 1.0; b += interval)
		{
			Point p1 = sam(o1, oa1, ob1, a, b);
			for (Real c = 0.0; c < 1.0; c += interval)
			{
				for (Real d = 0.0; d < 1.0; d += interval)
				{
					Point p2 = sam(o2, oa2, ob2, c, d);
					Real sqrD = glm::distance2(p1, p2);
					if (sqrD < minDistSqr)
					{
						minDistSqr = sqrD;
						pointPair = { p1, p2 };
					}
				}
			}
		}
	}
	return sqrt(minDistSqr);
}

Real maxSample(const Point& o1, const Point& a1, const Point& b1, const Point& o2, const Point& a2, const Point& b2, std::pair<Point, Point>& pointPair)
{
	Real maxDistSqr{ 0 };

	Vector oa1 = a1 - o1;
	Vector ob1 = b1 - o1;
	Vector oa2 = a2 - o2;
	Vector ob2 = b2 - o2;

	for (Real a = 0.0; a < 1.0; a += interval)
	{
		for (Real b = 0.0; b < 1.0; b += interval)
		{
			Point p1 = sam(o1, oa1, ob1, a, b);
			for (Real c = 0.0; c < 1.0; c += interval)
			{
				for (Real d = 0.0; d < 1.0; d += interval)
				{
					Point p2 = sam(o2, oa2, ob2, c, d);
					Real sqrD = glm::distance2(p1, p2);
					if (sqrD > maxDistSqr)
					{
						maxDistSqr = sqrD;
						pointPair = { p1, p2 };
					}
				}
			}
		}
	}
	return sqrt(maxDistSqr);
}

Real minDistSample(const OBB& a, const OBB& b, std::pair<Point, Point>& pointPair) {
	Real sqrDist{ 1000000 };

	constexpr std::array<std::array<unsigned, 3>, 6> faces = {
	std::array<unsigned, 3>{0, 2, 1},
	std::array<unsigned, 3>{0, 1, 4},
	std::array<unsigned, 3>{0, 4, 2},
	std::array<unsigned, 3>{1, 3, 5},
	std::array<unsigned, 3>{2, 6, 3},
	std::array<unsigned, 3>{4, 5, 6}
	};

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


	int index = 0;
	Real minDist{ 1000000 };

	std::mutex mtx;

#pragma omp parallel for
	//for (const auto& aface : faces)
	for (int i = 0; i < 6; ++i)
	{
		Point o1 = aPoints[faces[i][0]];
		Point a1 = aPoints[faces[i][1]];
		Point b1 = aPoints[faces[i][2]];
		for (const auto& bface : faces)
		{
			printf("sample process: %f\n", (index++) / 36.0f);
			Point o2 = bPoints[bface[0]];
			Point a2 = bPoints[bface[1]];
			Point b2 = bPoints[bface[2]];
			std::pair<Point, Point> curPair;
			Real dist = minSample(o1, a1, b1, o2, a2, b2, curPair);

			mtx.lock();
			if (dist < minDist)
			{
				minDist = dist;
				pointPair = curPair;
			}
			mtx.unlock();
			
		}
	}

	return minDist;
}


Real maxDistSample(const OBB& a, const OBB& b, std::pair<Point, Point>& pointPair) {
	Real sqrDist{ 0 };

	constexpr std::array<std::array<unsigned, 3>, 6> faces = {
	std::array<unsigned, 3>{0, 2, 1},
	std::array<unsigned, 3>{0, 1, 4},
	std::array<unsigned, 3>{0, 4, 2},
	std::array<unsigned, 3>{1, 3, 5},
	std::array<unsigned, 3>{2, 6, 3},
	std::array<unsigned, 3>{4, 5, 6}
	};

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


	int index = 0;
	Real maxDist{ 0 };

	std::mutex mtx;

#pragma omp parallel for
	//for (const auto& aface : faces)
	for (int i = 0; i < 6; ++i)
	{
		Point o1 = aPoints[faces[i][0]];
		Point a1 = aPoints[faces[i][1]];
		Point b1 = aPoints[faces[i][2]];
		for (const auto& bface : faces)
		{
			printf("sample process: %f\n", (index++) / 36.0f);
			Point o2 = bPoints[bface[0]];
			Point a2 = bPoints[bface[1]];
			Point b2 = bPoints[bface[2]];
			std::pair<Point, Point> curPair;
			Real dist = maxSample(o1, a1, b1, o2, a2, b2, curPair);

			mtx.lock();
			if (dist > maxDist)
			{
				maxDist = dist;
				pointPair = curPair;
			}
			mtx.unlock();

		}
	}

	return maxDist;
}