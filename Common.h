#pragma once
#include "Point.h"
#include "Vector.h"
#include <algorithm>

#define EPSILON 1E-4
#define TOL 1e-4

float distancePointSegment(const Point& p, const Point& a, const Point& b, std::pair<Point, Point>& pointPair) {
    Vector ab = b - a;
    float t = glm::dot(p - a, ab) / dot(ab, ab);

    if (t < 0.0f) t = 0.0f;
    else if (t > 1.0f) t = 1.0f;

    Point closest = a + t * ab;
    pointPair = { p, closest };
    return glm::dot(closest - p, closest - p);
}

__declspec(noinline)
float distancePointRect(const Point& p, const Point& o, const Point& a, const Point& b, std::pair<Point, Point>& pointPair) {
    float sqrDist = 1000000;
    
    Vector oa = a - o;
    Vector ob = b - o;
    Vector normal = glm::normalize(glm::cross(oa, ob));
    float d = glm::dot(normal, a);
    float t = glm::dot(normal, p) - d;
    Point closest = p - t * normal;

    Vector tmp = closest - o;
    float aProj = glm::dot(tmp, oa);
    float bProj = glm::dot(tmp, ob);
    if (aProj > 0 && aProj < dot(oa, oa) && bProj > 0 && bProj < dot(ob, ob))
    {
        pointPair = { p, closest };
        return glm::dot(closest - p, closest - p);
    }
    else
    {
        //std::array<std::pair<Point, Point>, 4> segs;
        //segs[0] = { o, a };
        //segs[1] = { a, o + oa + ob };
        //segs[2] = { o + oa + ob, b };
        //segs[3] = { b, o };
        //for (const auto& seg : segs)
        //{
        //    std::pair<Point, Point> curPair;
        //    float sqrD = distancePointSegment(p, seg.first, seg.second, curPair);
        //    if (sqrD < sqrDist)
        //    {
        //        sqrDist = sqrD;
        //        pointPair = curPair;
        //    }
        //}
        return sqrDist;
    }
}

float distancePointRect(const Point& p, const Point& p0, const Point& p1, const Point& p2, const Point& p3, std::pair<Point, Point>& pointPair) {
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

__declspec(noinline)
float distanceSegmentSegment(const Point& p1, const Point& q1, const Point& p2, const Point& q2, std::pair<Point, Point>& pointPair) {
    Vector d1 = q1 - p1; // Direction vector of segment S1
    Vector d2 = q2 - p2; // Direction vector of segment S2
    Vector r = p1 - p2;
    float a = glm::dot(d1, d1); // Squared length of segment S1, always nonnegative
    float e = glm::dot(d2, d2); // Squared length of segment S2, always nonnegative
    float f = glm::dot(d2, r);

    Point c1, c2;
    float s, t;

    // Check if either or both segments degenerate into points
    if (a <= EPSILON && e <= EPSILON) {
        // Both segments degenerate into points
        s = t = 0.0f;
        c1 = p1;
        c2 = p2;
        return glm::dot(c1 - c2, c1 - c2);
    }
    if (a <= EPSILON) {
        // First segment degenerates into a point
        s = 0.0f;
        t = f / e; // s = 0 => t = (b*s + f) / e = f / e
        t = std::clamp(t, 0.0f, 1.0f);
    }
    else {
        float c = glm::dot(d1, r);
        if (e <= EPSILON) {
            // Second segment degenerates into a point
            t = 0.0f;
            s = std::clamp(-c / a, 0.0f, 1.0f); // t = 0 => s = (b*t - c) / a = -c / a
        }
        else {
            // The general nondegenerate case starts here
            float b = glm::dot(d1, d2);
            float denom = a * e - b * b; // Always nonnegative

            // If segments not parallel, compute closest point on L1 to L2, and
            // clamp to segment S1. Else pick arbitrary s (here 0)
            if (denom != 0.0f) {
                s = std::clamp((b * f - c * e) / denom, 0.0f, 1.0f);
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
                s = std::clamp(-c / a, 0.0f, 1.0f);
            }
            else if (t > 1.0f) {
                t = 1.0f;
                s = std::clamp((b - c) / a, 0.0f, 1.0f);
            }
        }
    }

    c1 = p1 + d1 * s;
    c2 = p2 + d2 * t;
    pointPair = { c1, c2 };
    return glm::dot(c1 - c2, c1 - c2);
}


float distanceSegmentRect(const Point& p, const Point& q, const Point& a, const Point& b, const Point& c, const Point& d, std::pair<Point, Point>& pointPair) {
    float sqrDist{ 1000 };
    
    std::pair<Point, Point> curPair;
    float sqrD = distancePointRect(p, a, b, c, d, curPair);
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