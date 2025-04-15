#pragma once
#include "Point.h"
#include "Vector.h"
#include <vector>

struct OBB {
    Point c;     // OBB center point
    Vector u[3]; // Local x-, y-, and z-axes
    Vector e;    // Positive halfwidth extents of OBB along each axis

    Point getPoint(unsigned int index) const {
        return c
            + u[0] * ((index & 1) ? e.x : -e.x)   // Use positive or negative x-axis extent
            + u[1] * ((index & 2) ? e.y : -e.y)   // Use positive or negative y-axis extent
            + u[2] * ((index & 4) ? e.z : -e.z);  // Use positive or negative z-axis extent
    }
};
