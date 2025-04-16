#pragma once
#include "OBB.h"
#include "Common.h"
#include <array>

float distanceRectRect(const OBB& a, const OBB& b, std::pair<Point, Point>& pointPair) {
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

	constexpr std::array<std::array<unsigned, 3>, 6> faces = {
		std::array<unsigned, 3>{0, 2, 1},
		std::array<unsigned, 3>{0, 1, 4},
		std::array<unsigned, 3>{0, 4, 2},
		std::array<unsigned, 3>{1, 3, 5},
		std::array<unsigned, 3>{2, 6, 3},
		std::array<unsigned, 3>{4, 5, 6}
	};

	std::pair<Point, Point> curPair;
#ifdef DEBUG_DISTANCE
	std::cout << "-------------Distance(Point of a, Rect of b)-------------" << std::endl;
#endif
	int cnt{ 0 };
	for (const auto& p : aPoints)
	{
		for (const auto& face : faces)
		{
			float sqrD = distancePointRect(p, bPoints[face[0]], bPoints[face[1]], bPoints[face[2]], curPair);
#ifdef DEBUG_DISTANCE
			std::cout << sqrt(sqrD) << " --- Point " << cnt << ", Rect[Point" << face[0] << ", Point " << face[1] << ", Point " << face[2] << "]" << std::endl;
#endif
			if (sqrD < sqrDist)
			{
				sqrDist = sqrD;
				pointPair = curPair;
			}
		}
		cnt++;
	}
	cnt = 0;
#ifdef DEBUG_DISTANCE
	std::cout << "-------------Distance(Point of b, Rect of a)-------------" << std::endl;
#endif
	for (const auto& p : bPoints)
	{
		for (const auto& face : faces)
		{
			float sqrD = distancePointRect(p, aPoints[face[0]], aPoints[face[1]], aPoints[face[2]], curPair);
#ifdef DEBUG_DISTANCE
			std::cout << sqrt(sqrD) << " --- Point " << cnt << ", Rect[Point" << face[0] << ", Point " << face[1] << ", Point " << face[2] << "]" << std::endl;
#endif
			if (sqrD < sqrDist)
			{
				sqrDist = sqrD;
				std::swap(curPair.first, curPair.second);
				pointPair = curPair;
			}
		}
		cnt++;
	}
#ifdef DEBUG_DISTANCE
	std::cout << "-------------Distance(Edge of a, Edge of b)-------------" << std::endl;
#endif
	constexpr std::array<std::pair<unsigned int, unsigned int>, 12> edges = {
		std::make_pair(0, 1), std::make_pair(0, 2), std::make_pair(0, 4),
		std::make_pair(1, 3), std::make_pair(1, 5), std::make_pair(2, 3),
		std::make_pair(2, 6), std::make_pair(3, 7), std::make_pair(4, 5),
		std::make_pair(4, 6), std::make_pair(5, 7), std::make_pair(6, 7)
	};
	for (const auto& ledge : edges) {
		for (const auto& redge : edges) {
			float sqrD = distanceSegmentSegment(a.getPoint(ledge.first), a.getPoint(ledge.second), b.getPoint(redge.first), b.getPoint(redge.second), curPair);
#ifdef DEBUG_DISTANCE
			std::cout << sqrt(sqrD) << " --- Seg[Point" << ledge.first << ", Point " << ledge.second << "], Seg[Point " << redge.first << ", Point " << ledge.second << "]" << std::endl;
#endif
			if (sqrD < sqrDist)
			{
				sqrDist = sqrD;
				pointPair = curPair;
			}
		}
	}
	return sqrDist;
}