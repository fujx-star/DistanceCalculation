#pragma once
#include "OBB.h"
#include "Common.h"
#include <array>
#include <limits>
#include <iostream>
#include <iomanip>
#include <glm/gtx/string_cast.hpp>
#include "Matrix.h"

Vector matmul(const Real* weights, const Point* points, uint32_t size)
{
	Vector res(0.0f);
	for (int i = 0; i < 3; i++)
	{
		for (int k = 0; k < size; k++)
		{
			res[i] += weights[k] * points[k][i];
		}
	}
	return res;
}

std::pair<Point, Real> supportFunc(const OBB& obb, const Vector& dir)
{
	Real dx = glm::dot(dir, obb.u[0]) * obb.e.x;
	Real dy = glm::dot(dir, obb.u[1]) * obb.e.y;
	Real dz = glm::dot(dir, obb.u[2]) * obb.e.z;

	Real signX = dx > 0 ? 1 : -1;
	Real signY = dy > 0 ? 1 : -1;
	Real signZ = dz > 0 ? 1 : -1;

	Point p = obb.c
		+ obb.u[0] * signX * obb.e.x
		+ obb.u[1] * signY * obb.e.y
		+ obb.u[2] * signZ * obb.e.z;
	return { p, glm::dot(obb.c, dir) + dx * signX + dy * signY + dz * signZ};

	//Real d = std::numeric_limits<Real>::lowest();
	//Point res;
	//for (int i = 0; i < 8; i++)
	//{
	//	Point p = obb.getPoint(i);
	//	Real dt = glm::dot(dir, p);
	//	if (dt > d)
	//	{
	//		d = dt;
	//		res = p;
	//	}
	//}
	//return { res, d };
}

std::pair<Point, Real> supportMinkowskiDiff(const OBB& a, const OBB& b, const Vector& dir)
{
	auto [p1, dist1] = supportFunc(a, dir);
	//std::cout << glm::to_string(p1) << ", " << dist1 << std::endl;

	auto [p2, dist2] = supportFunc(b, -dir);
	//std::cout << glm::to_string(p2) << ", " << dist2 << std::endl;


	return { p1 - p2, dist1 + dist2 };
}

uint32_t S1D(const Point* simplex, Point* newSimplex, Real* newWeights) {
	//printf("-----------S1D start-----------\n");
	//for (int i = 0; i < 2; ++i) {
	//	std::cout << glm::to_string(simplex[i]) << std::endl;
	//}

	if (glm::all(glm::equal(simplex[1], simplex[0]))) {
		newSimplex[0] = simplex[1];
		newWeights[0] = 1.0f;
		return 1;
	}

	Vector t = simplex[1] - simplex[0];
	Vector po = -glm::dot(simplex[1], t) / glm::dot(t, t) * t + simplex[1];
	Real u_max = 0.0f;
	int Index = 0;

	for (int i = 0; i < 3; ++i) {
		Real u = simplex[0][i] - simplex[1][i];
		if (std::abs(u) > std::abs(u_max)) {
			u_max = u;
			Index = i;
		}
	}

	glm::vec2 C2(0.0f);
	for (int j = 0; j < 2; ++j) {
		Real sign = ((j + 1) % 2 == 0) ? 1.0f : -1.0f;
		C2[j] = sign * (simplex[(j + 1) % 2][Index] - po[Index]);
	}

	if ((u_max > 0 && glm::all(glm::greaterThan(C2, glm::vec2(0.0f)))) ||
		(u_max < 0 && glm::all(glm::lessThan(C2, glm::vec2(0.0f))))) {
		newSimplex[0] = simplex[0];
		newSimplex[1] = simplex[1];
		newWeights[0] = C2[0] / u_max;
		newWeights[1] = C2[1] / u_max;
		return 2;
	}
	else {
		if (glm::length(simplex[0]) < glm::length(simplex[1])) {
			newSimplex[0] = simplex[0];
			newWeights[0] = 1.0f;
		}
		else {
			newSimplex[0] = simplex[1];
			newWeights[0] = 1.0f;
		}

		return 1;
	}
}


uint32_t S2D(const Point* simplex, Point* newSimplex, Real* newWeights) {
	//printf("-----------S2D start-----------\n");
	//for (int i = 0; i < 3; ++i) {
	//	std::cout << glm::to_string(simplex[i]) << std::endl;
	//}

	Vector n = glm::cross(simplex[1] - simplex[0], simplex[2] - simplex[0]);
	Vector po = (glm::dot(simplex[0], n) / glm::dot(n, n)) * n;

	Real u_max = 0.0f;
	int J = 0;

#if 0
	for (int i = 0; i < 3; ++i) {
		Real sign = (i % 2 == 0) ? 1.0f : -1.0f;
		int a = (i + 1) % 3;
		int b = (i + 2) % 3;
		Real u = sign * (
			simplex[0][a] * simplex[1][b] +
			simplex[1][a] * simplex[2][b] +
			simplex[2][a] * simplex[0][b] -
			simplex[1][a] * simplex[0][b] -
			simplex[2][a] * simplex[1][b] -
			simplex[0][a] * simplex[2][b]
			);
		if (std::abs(u) > std::abs(u_max)) {
			u_max = u;
			J = i;
		}
	}
#else
	{
		Real u;
		u = simplex[0][1] * simplex[1][2] +
			simplex[1][1] * simplex[2][2] +
			simplex[2][1] * simplex[0][2] -
			simplex[1][1] * simplex[0][2] -
			simplex[2][1] * simplex[1][2] -
			simplex[0][1] * simplex[2][2];
		if (std::abs(u) > std::abs(u_max)) {
			u_max = u;
			J = 0;
		}

		u = simplex[0][2] * simplex[1][0] +
			simplex[1][2] * simplex[2][0] +
			simplex[2][2] * simplex[0][0] -
			simplex[1][2] * simplex[0][0] -
			simplex[2][2] * simplex[1][0] -
			simplex[0][2] * simplex[2][0];
		u = -u;
		if (std::abs(u) > std::abs(u_max)) {
			u_max = u;
			J = 1;
		}

		u = simplex[0][0] * simplex[1][1] +
			simplex[1][0] * simplex[2][1] +
			simplex[2][0] * simplex[0][1] -
			simplex[1][0] * simplex[0][1] -
			simplex[2][0] * simplex[1][1] -
			simplex[0][0] * simplex[2][1];
		if (std::abs(u) > std::abs(u_max)) {
			u_max = u;
			J = 2;
		}
	}
#endif

	int x = 0, y = 1;
	if (J == 0) {
		x = 1, y = 2;
	}
	else if (J == 1) {
		x = 0, y = 2;
	}

	Vector C3 = {
		po[x] * simplex[1][y] + po[y] * simplex[2][x] + simplex[1][x] * simplex[2][y] - po[x] * simplex[2][y] - po[y] * simplex[1][x] - simplex[2][x] * simplex[1][y],
		po[x] * simplex[2][y] + po[y] * simplex[0][x] + simplex[2][x] * simplex[0][y] - po[x] * simplex[0][y] - po[y] * simplex[2][x] - simplex[0][x] * simplex[2][y],
		po[x] * simplex[0][y] + po[y] * simplex[1][x] + simplex[0][x] * simplex[1][y] - po[x] * simplex[1][y] - po[y] * simplex[0][x] - simplex[1][x] * simplex[0][y],
	};

	if ((u_max > 0 && glm::all(glm::greaterThan(C3, Vector(0.0f)))) ||
		(u_max < 0 && glm::all(glm::lessThan(C3, Vector(0.0f))))) {
		Real tmpr = 1.0f / u_max;
		newSimplex[0] = simplex[0];
		newSimplex[1] = simplex[1];
		newSimplex[2] = simplex[2];
		newWeights[0] = C3[0] * tmpr;
		newWeights[1] = C3[1] * tmpr;
		newWeights[2] = C3[2] * tmpr;
		return 3;
	}

	Real d = std::numeric_limits<Real>::max();

	Point simplex1D[3], newSimplex1D[3];
	Real weight1D[3], newWeight1D[3];
	uint32_t size1D;
	uint32_t size;

	for (int j = 0; j < 3; ++j) {
		if ((u_max >= 0 && -C3[j] >= 0) || (u_max <= 0 && -C3[j] <= 0)) {
			for (int i = 0; i < j; ++i) {
				simplex1D[i] = simplex[i];
			}
			for (int i = j; i < 2; ++i) {
				simplex1D[i] = simplex[i + 1];
			}

			size1D = S1D(simplex1D, newSimplex1D, newWeight1D);
			Vector closestVector = matmul(newWeight1D, newSimplex1D, size1D);
			Real d_astrix = glm::length(closestVector);
			if (d_astrix < d) {
				newSimplex[0] = newSimplex1D[0];
				newSimplex[1] = newSimplex1D[1];
				newSimplex[2] = newSimplex1D[2];
				newWeights[0] = newWeight1D[0];
				newWeights[1] = newWeight1D[1];
				newWeights[2] = newWeight1D[2];
				size = size1D;
				d = d_astrix;
			}
		}
	}

	return size;
}

uint32_t S3D(const Point* simplex, Point* newSimplex, Real* newWeights) {
	//printf("-----------S3D start-----------\n");
	//for (int i = 0; i < 4; ++i) {
	//	std::cout << glm::to_string(simplex[i]) << std::endl;
	//}

	Matrix4 M{
		Vector4(simplex[0], 1.0f),
		Vector4(simplex[1], 1.0f),
		Vector4(simplex[2], 1.0f),
		Vector4(simplex[3], 1.0f) };

	Real detM = 0.0f;
	Vector4 C4;

	// 代数余子式
	for (int j = 0; j < 4; ++j) {
		Matrix3 minorMat;
		int colIndex = 0;

		for (int col = 0; col < 4; ++col) {
			if (col == j) continue;  // skip the j-th column

			// Copy first 3 rows of this column into minorMat
			minorMat[colIndex][0] = M[col][0];
			minorMat[colIndex][1] = M[col][1];
			minorMat[colIndex][2] = M[col][2];
			colIndex++;
		}

		Real sign = ((j + 1 + 4) % 2 == 0) ? 1.0f : -1.0f;
		C4[j] = sign * glm::determinant(minorMat);
		detM += C4[j];
	}

	if ((detM > 0 && glm::all(glm::greaterThan(C4, Vector4(0.0f)))) ||
		(detM < 0 && glm::all(glm::lessThan(C4, Vector4(0.0f))))) {
		Real detMr = 1.0f / detM;
		for (int i = 0; i < 4; ++i) {
			newSimplex[i] = simplex[i];
		}
		for (int i = 0; i < 4; ++i) {
			newWeights[i] = C4[i] * detMr;
		}

		return 4;
	}

	Real d = std::numeric_limits<Real>::max();
	Point simplex2D[3], newSimplex2D[3];
	Real weight2D[3], newWeight2D[3];
	uint32_t size2D;
	uint32_t size;

	for (int j = 0; j < 4; ++j) {
		size2D = 3;
		if ((detM >= 0 && -C4[j] >= 0) || (detM <= 0 && -C4[j] <= 0)) {

			for (int i = 0; i < j; ++i) {
				simplex2D[i] = simplex[i];
			}
			for (int i = j; i < 3; ++i) {
				simplex2D[i] = simplex[i + 1];
			}

			size2D = S2D(simplex2D, newSimplex2D, newWeight2D);
			Vector closestVector = matmul(newWeight2D, newSimplex2D, size2D);
			Real d_astrix = glm::length(closestVector);
			if (d_astrix < d) {
				newSimplex[0] = newSimplex2D[0];
				newSimplex[1] = newSimplex2D[1];
				newSimplex[2] = newSimplex2D[2];
				newWeights[0] = newWeight2D[0];
				newWeights[1] = newWeight2D[1];
				newWeights[2] = newWeight2D[2];
				size = size2D;
				d = d_astrix;
			}
		}
	}

	return size;
}

uint32_t SignedVolumes(Point* simplex, Point* points, Real* weights, uint32_t& size)
{
	if (size == 4)
	{
		return S3D(simplex, points, weights);
	}
	else if (size == 3)
	{
		return S2D(simplex, points, weights);
	}
	else if (size == 2)
	{
		return S1D(simplex, points, weights);
	}
	else
	{
		points[0] = simplex[0];
		weights[0] = 1.0f;
		return 1;
	}
}


Real distanceGJK(const OBB& a, const OBB& b, std::pair<Point, Point>& pointPair)
{
	Vector dir = a.c - b.c;
	auto [newPoint, dist] = supportMinkowskiDiff(a, b, -dir);

	Point simplex[4];
	simplex[0] = newPoint;
	simplex[1] = dir;

	Point retFirst[4];
	Real retSecond[4];
	uint32_t size = 2;

	while (true)
	{
		size = SignedVolumes(simplex, retFirst, retSecond, size);
		dir = matmul(retSecond, retFirst, size);
		auto [newPoint, dist] = supportMinkowskiDiff(a, b, -dir);
		Real vk_square = glm::dot(dir, dir);
		Real gk = vk_square + dist;
		if (gk < TOL || size == 4)
		{
			return glm::sqrt(vk_square);
		}

		memmove(retFirst + 1, retFirst, sizeof(Vector) * (size));
		retFirst[0] = newPoint;
		size += 1;
		memcpy(simplex, retFirst, sizeof(Vector) * size);
	}

	return 0.0f;
}