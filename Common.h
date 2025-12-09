#pragma once
#include "Point.h"
#include "Vector.h"
#include <algorithm>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>

#define EPSILON 1E-4
#define TOL 1e-4

Real distancePointSegment(const Point& p, const Point& a, const Point& b, std::pair<Point, Point>& pointPair) {
	Vector ab = b - a;
	Real t = glm::dot(p - a, ab) / dot(ab, ab);

	if (t < 0.0f) t = 0.0f;
	else if (t > 1.0f) t = 1.0f;

	Point closest = a + t * ab;
	pointPair = { p, closest };
	return glm::dot(closest - p, closest - p);
}

//__declspec(noinline)
Real distancePointRect(const Point& p, const Point& o, const Point& a, const Point& b, std::pair<Point, Point>& pointPair) {

	Vector oa = a - o;
	Vector ob = b - o;
	Vector normal = glm::normalize(glm::cross(oa, ob));
	Real d = glm::dot(normal, a);
	Real t = glm::dot(normal, p) - d;
	Point closest = p - t * normal;

	// closest point is inside the rectangle plane
	Vector tmp = closest - o;
	Real aProj = glm::dot(tmp, oa);
	Real bProj = glm::dot(tmp, ob);
	if (aProj > 0 && aProj < dot(oa, oa) && bProj > 0 && bProj < dot(ob, ob))
	{
		pointPair = { p, closest };
		return glm::dot(closest - p, closest - p);
	}
	else
	{
		Real oa2 = glm::dot(oa, oa);
		Real ob2 = glm::dot(ob, ob);
		Vector tmp = closest - o;
		Real u = glm::dot(tmp, oa) / oa2;
		Real v = glm::dot(tmp, ob) / ob2;

		// clamp µ½ [0,1]
		u = std::clamp<Real>(u, 0.0f, 1.0f);
		v = std::clamp<Real>(v, 0.0f, 1.0f);

		Point closestNew = o + u * oa + v * ob;
		pointPair = { p, closestNew };
		return glm::dot(closestNew - p, closestNew - p);
	}
}

// not finished
//__declspec(noinline)
Real distancePointRect2(
	const Point& p,
	const Point& o1, const Point& a1, const Point& b1,
	const Point& o2, const Point& a2, const Point& b2,
	const Vector& normal,
	std::pair<Point, Point>& pointPair) {
	Real sqrDist = 1000000;

	Vector oa = a1 - o1;
	Vector ob = b1 - o1;
	Real oa2 = glm::dot(oa, oa);
	Real ob2 = glm::dot(ob, ob);
	//Vector normal = glm::normalize(glm::cross(oa, ob));
	Real pProj = glm::dot(normal, p);

	Real d1 = glm::dot(normal, a1);
	Real t1 = pProj - d1;
	Point closest1 = p - t1 * normal;

	Real d2 = glm::dot(normal, a2);
	Real t2 = pProj - d2;
	Point closest2 = p - t2 * normal;

	{
		Vector closestVec = closest1 - o1;

		Real aProj = glm::dot(closestVec, oa);
		Real bProj = glm::dot(closestVec, ob);
		Real sqrD = glm::distance2(closest1, p);

		if (aProj > 0 && aProj < dot(oa, oa) && bProj > 0 && bProj < dot(ob, ob) && sqrD < sqrDist)
		{
			pointPair = { p, closest1 };
			sqrDist = sqrD;
		}
		else
		{
			Vector tmp = closest1 - o1;
			Real s = glm::dot(tmp, oa) / oa2;
			Real t = glm::dot(tmp, ob) / ob2;
			s = std::clamp<Real>(s, 0.0f, 1.0f);
			t = std::clamp<Real>(t, 0.0f, 1.0f);

			Point closest1New = o1 + s * oa + t * ob;
			pointPair = { p, closest1New };
			sqrDist += glm::dot(closest1New - closest1, closest1New - closest1);
		}
	}
	{
		Vector closestVec = closest2 - o2;

		Real aProj = glm::dot(closestVec, oa);
		Real bProj = glm::dot(closestVec, ob);
		Real sqrD = glm::distance2(closest2, p);

		if (aProj > 0 && aProj < dot(oa, oa) && bProj > 0 && bProj < dot(ob, ob) && sqrD < sqrDist)
		{
			pointPair = { p, closest2 };
			sqrDist = sqrD;
		}
		else
		{
			Vector tmp = closest2 - o2;
			Real u = glm::dot(tmp, oa) / oa2;
			Real v = glm::dot(tmp, ob) / ob2;
			u = std::clamp<Real>(u, 0.0f, 1.0f);
			v = std::clamp<Real>(v, 0.0f, 1.0f);

			Point closest2New = o2 + u * oa + v * ob;
			pointPair = { p, closest2New };
			sqrDist += glm::dot(closest2New - p, closest2New - closest2);
		}
	}

	return sqrDist;
}

//__declspec(noinline)
Real distancePointRect(const Point& p, const Point& p0, const Point& p1, const Point& p2, const Point& p3, std::pair<Point, Point>& pointPair) {
	Vector ab = p1 - p0;
	Vector ac = p2 - p0;
	Vector ad = p3 - p0;

	Vector v0 = glm::cross(ab, ac);
	Vector v1 = glm::cross(ab, ad);
	Vector v2 = glm::cross(ac, ad);
	if (glm::dot(v0, v1) > 0)
	{
		if (glm::dot(v0, v2) > 0)
		{
			return distancePointRect(p, p0, p1, p3, pointPair);
		}
		else
		{
			return distancePointRect(p, p0, p1, p2, pointPair);
		}
	}
	else
	{
		return distancePointRect(p, p0, p2, p3, pointPair);
	}
}

//__declspec(noinline)
Real distanceSegmentSegment(const Point& p1, const Point& q1, const Point& p2, const Point& q2, std::pair<Point, Point>& pointPair) {
	Vector d1 = q1 - p1; // Direction vector of segment S1
	Vector d2 = q2 - p2; // Direction vector of segment S2
	Vector r = p1 - p2;
	Real a = glm::dot(d1, d1); // Squared length of segment S1, always nonnegative
	Real e = glm::dot(d2, d2); // Squared length of segment S2, always nonnegative
	Real f = glm::dot(d2, r);

	Point c1, c2;
	Real s, t;

	// Check if either or both segments degenerate into points
	if (a <= EPSILON && e <= EPSILON) {
		// Both segments degenerate into points
		s = t = 0.0f;
		c1 = p1;
		c2 = p2;
		pointPair = { c1, c2 };
		return glm::dot(c1 - c2, c1 - c2);
	}
	if (a <= EPSILON) {
		// First segment degenerates into a point
		s = 0.0f;
		t = f / e; // s = 0 => t = (b*s + f) / e = f / e
		t = std::clamp<Real>(t, 0.0f, 1.0f);
	}
	else {
		Real c = glm::dot(d1, r);
		if (e <= EPSILON) {
			// Second segment degenerates into a point
			t = 0.0f;
			s = std::clamp<Real>(-c / a, 0.0f, 1.0f); // t = 0 => s = (b*t - c) / a = -c / a
		}
		else {
			// The general nondegenerate case starts here
			Real b = glm::dot(d1, d2);
			Real denom = a * e - b * b; // Always nonnegative

			// If segments not parallel, compute closest point on L1 to L2, and
			// clamp to segment S1. Else pick arbitrary s (here 0)
			if (denom != 0.0f) {
				s = std::clamp<Real>((b * f - c * e) / denom, 0.0f, 1.0f);
			}
			else s = 0.0f;

			// Compute point on L2 closest to S1(s) using
			// t = glm::dot((P1+D1*s)-P2,D2) / glm::dot(D2,D2) = (b*s + f) / e
			t = (b * s + f) / e;

			// If t in [0,1] done. Else clamp t, recompute s for the new value
			// of t using s = glm::dot((P2+D2*t)-P1,D1) / glm::dot(D1,D1)= (t*b - c) / a
			// and clamp s to [0, 1]
			if (t < 0.0f) {
				t = 0.0f;
				s = std::clamp<Real>(-c / a, 0.0f, 1.0f);
			}
			else if (t > 1.0f) {
				t = 1.0f;
				s = std::clamp<Real>((b - c) / a, 0.0f, 1.0f);
			}
		}
	}

	c1 = p1 + d1 * s;
	c2 = p2 + d2 * t;
	pointPair = { c1, c2 };
	return glm::dot(c1 - c2, c1 - c2);
}

//__declspec(noinline)
Real distanceSegment4Segment4(
	const std::array<std::pair<Point, Point>, 4>& segsA,
	const std::array<std::pair<Point, Point>, 4>& segsB,
	std::pair<Point, Point>& pointPair) {
	Real sqrDist = 1000000;

	Vector d1 = segsA[0].second - segsA[0].first;
	Vector d2 = segsB[0].second - segsB[0].first;
	Real a = glm::dot(d1, d1);
	Real e = glm::dot(d2, d2);
	Real b = glm::dot(d1, d2);
	Real denom = a * e - b * b; // Always nonnegative

	Real ra = 1.0f / a;
	Real rd = 1.0f / denom;
	Real re = 1 / e;
	Point c1, c2;
	Real s, t;

	if (a <= EPSILON && e <= EPSILON) {
		// Both segments degenerate into points
		s = t = 0.0f;
		pointPair = { segsA[0].first, segsB[0].first };
		return glm::dot(c1 - c2, c1 - c2);
	}

	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			Vector r = segsA[i].first - segsB[j].first;
			Real f = glm::dot(d2, r);

			if (a <= EPSILON) {
				// First segment degenerates into a point
				s = 0.0f;
				t = f * re; // s = 0 => t = (b*s + f) / e = f / e
				t = std::clamp<Real>(t, 0.0f, 1.0f);
			}
			else {
				Real c = glm::dot(d1, r);
				if (e <= EPSILON) {
					t = 0.0f;
					s = std::clamp<Real>(-c * ra, 0.0f, 1.0f); // t = 0 => s = (b*t - c) / a = -c / a
				}
				else {
					// If segments not parallel, compute closest point on L1 to L2, and
					// clamp to segment S1. Else pick arbitrary s (here 0)
					if (denom != 0.0f) [[likely]] {
						s = std::clamp<Real>((b * f - c * e) * rd, 0.0f, 1.0f);
					}
					else {
						s = 0.0f;
					}

					// Compute point on L2 closest to S1(s) using
					// t = glm::dot((P1+D1*s)-P2,D2) / glm::dot(D2,D2) = (b*s + f) / e
					t = (b * s + f) * re;
					//t = s + b + f + re;

					// If t in [0,1] done. Else clamp t, recompute s for the new value
					// of t using s = glm::dot((P2+D2*t)-P1,D1) / glm::dot(D1,D1)= (t*b - c) / a
					// and clamp s to [0, 1]
					if (t < 0.0f) {
						t = 0.0f;
						s = std::clamp<Real>(-c * ra, 0.0f, 1.0f);
					}
					else if (t > 1.0f) {
						t = 1.0f;
						s = std::clamp<Real>((b - c) * ra, 0.0f, 1.0f);
					}
				}
			}

			c1 = segsA[i].first + d1 * s;
			c2 = segsB[j].first + d2 * t;
			Real sqrD = glm::dot(c1 - c2, c1 - c2);
			//std::cout << i << " " << j << " : " << sqrD << std::endl;
			if (sqrD < sqrDist)
			{
				pointPair = { c1, c2 };
				sqrDist = sqrD;
			}
		}
	}

	return sqrDist;
}

//__declspec(noinline)
Real distanceSegmentRect(const Point& p, const Point& q, const Point& a, const Point& b, const Point& c, const Point& d, std::pair<Point, Point>& pointPair) {
	Real sqrDist{ 1000 };

	std::pair<Point, Point> curPair;
	Real sqrD = distancePointRect(p, a, b, c, d, curPair);
	if (sqrD < sqrDist)
	{
		sqrDist = sqrD;
		pointPair = curPair;
	}
	sqrD = distancePointRect(q, a, b, c, d, curPair);
	if (sqrD < sqrDist)
	{
		sqrDist = sqrD;
		pointPair = curPair;
	}
	std::array<std::pair<Point, Point>, 4> segs;
	segs[0] = { a, b };
	segs[1] = { b, c };
	segs[2] = { c, d };
	segs[3] = { d, a };
	for (const auto& seg : segs)
	{
		sqrD = distanceSegmentSegment(p, q, seg.first, seg.second, curPair);
		if (sqrD < sqrDist)
		{
			sqrDist = sqrD;
			pointPair = curPair;
		}
	}
	return sqrDist;
}