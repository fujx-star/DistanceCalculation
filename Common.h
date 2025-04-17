#pragma once
#include "Point.h"
#include "Vector.h"
#include <algorithm>

#define EPSILON 1E-4
#define TOL 1e-4

float distancePointSegment(const Point& p, const Point& a, const Point& b, std::pair<Point, Point>& pointPair) {
    Vector ab = b - a;
    float t = glm::dot(p - a, ab);

    if (t < 0.0f) t = 0.0f;
    else if (t > 1.0f) t = 1.0f;

    Point closest = a + t * ab;
    pointPair = { p, closest };
    return glm::dot(closest - p, closest - p);
}

float distancePointRect(const Point& p, const Point& o, const Point& a, const Point& b, std::pair<Point, Point>& pointPair) {
    Vector oa = a - o;
    Vector ob = b - o;
    Vector normal = glm::normalize(glm::cross(oa, ob));
    float d = glm::dot(normal, a);
    float t = (glm::dot(normal, p) - d);
    Point closest = p - t * normal;

    float aProj = glm::dot(closest - o, oa);
    float bProj = glm::dot(closest - o, ob);
    if (aProj > 0 && aProj < dot(oa, oa) && bProj > 0 && bProj < dot(ob, ob))
    {
        pointPair = { p, closest };
        return glm::dot(closest - p, closest - p);
    }
    else
    {
        return 1000000;
    }
}


float distanceSegmentSegment(const Vector& p1, const Vector& q1, const Vector& p2, const Vector& q2, std::pair<Point, Point>& pointPair) {
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