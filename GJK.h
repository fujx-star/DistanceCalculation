#pragma once
#include "OBB.h"
#include "Common.h"
#include <array>
#include <limits>
#include <iostream>
#include <iomanip>

Vector matmul(const std::vector<float>& Lambda, const std::vector<Point>& W)
{
	glm::vec3 res(0.0f);
	for (int i = 0; i < 3; i++)
	{
		for (int k = 0; k < Lambda.size(); k++)
		{
			res[i] += Lambda[k] * W[k][i];
		}
	}
	return res;
}

std::pair<Point, float> supportFunc(const OBB& obb, const Vector& dir)
{
	float d = std::numeric_limits<float>::lowest();
	Point res;
	for (int i = 0; i < 8; i++)
	{
		Point p = obb.getPoint(i);
		float dt = glm::dot(dir, p);
		if (dt > d)
		{
			d = dt;
			res = p;
		}
	}
	return { res, d };
}

std::pair<Point, float> supportMinkowskiDiff(const OBB& a, const OBB& b, const Vector& dir)
{
	auto [p1, dist1] = supportFunc(a, dir);
	auto [p2, dist2] = supportFunc(b, -dir);
	return { p1 - p2, dist1 + dist2 };
}

std::pair<std::vector<glm::vec3>, std::vector<float>> S1D(const std::vector<glm::vec3>& simplex) {
	if (glm::all(glm::equal(simplex[1], simplex[0]))) {
		return { { simplex[1] }, {1.0f} };
	}

	glm::vec3 t = simplex[1] - simplex[0];
	glm::vec3 po = -glm::dot(simplex[1], t) / glm::dot(t, t) * t + simplex[1];
	float u_max = 0.0f;
	int Index = 0;

	for (int i = 0; i < 3; ++i) {
		float u = simplex[0][i] - simplex[1][i];
		if (std::abs(u) > std::abs(u_max)) {
			u_max = u;
			Index = i;
		}
	}

	glm::vec2 C2(0.0f);
	for (int j = 0; j < 2; ++j) {
		float sign = ((j + 1) % 2 == 0) ? 1.0f : -1.0f;
		C2[j] = sign * (simplex[(j + 1) % 2][Index] - po[Index]);
	}

	if ((u_max > 0 && glm::all(glm::greaterThan(C2, glm::vec2(0.0f)))) ||
		(u_max < 0 && glm::all(glm::lessThan(C2, glm::vec2(0.0f))))) {
		std::pair<std::vector<glm::vec3>, glm::vec3> res;
		return { simplex, {C2[0] / u_max, C2[1] / u_max} };
	}
	else {
		if (glm::length(simplex[0]) < glm::length(simplex[1])) {
			return { {simplex[0]}, {1.0f} };
		}
		else {
			return { {simplex[1]}, {1.0f} };
		}
	}
}


std::pair<std::vector<glm::vec3>, std::vector<float>> S2D(const std::vector<glm::vec3>& simplex) {
	glm::vec3 n = glm::cross(simplex[1] - simplex[0], simplex[2] - simplex[0]);
	glm::vec3 po = (glm::dot(simplex[0], n) / glm::dot(n, n)) * n;

	float u_max = 0.0f;
	int J = 0;
	for (int i = 0; i < 3; ++i) {
		float sign = (i % 2 == 0) ? 1.0f : -1.0f;
		float u = sign * (
			simplex[0][(i + 1) % 3] * simplex[1][(i + 2) % 3] +
			simplex[1][(i + 1) % 3] * simplex[2][(i + 2) % 3] +
			simplex[2][(i + 1) % 3] * simplex[0][(i + 2) % 3] -
			simplex[1][(i + 1) % 3] * simplex[0][(i + 2) % 3] -
			simplex[2][(i + 1) % 3] * simplex[1][(i + 2) % 3] -
			simplex[0][(i + 1) % 3] * simplex[2][(i + 2) % 3]
			);
		if (std::abs(u) > std::abs(u_max)) {
			u_max = u;
			J = i;
		}
	}

	std::vector<int> indices = { 0, 1, 2 };
	indices.erase(indices.begin() + J);
	int x = indices[0];
	int y = indices[1];

	glm::vec3 C3(0.0f);
	//std::array<float, 3> poXsimpY;
	//std::array<float, 3> poYsimpX;
	//std::array<std::array<float, 2>, 3> simpXsimpY;
	//for (int i = 0; i < 3; ++i) {
	//	poXsimpY[i] = (po[x] * simplex[i][y]);
	//	poYsimpX[i] = (po[y] * simplex[i][x]);
	//	simpXsimpY[i][0] = (simplex[i][x] * simplex[(i + 1) % 3][y]);
	//	simpXsimpY[i][1] = (simplex[i][x] * simplex[(i - 1) % 3][y]);
	//}
	for (int j = 0; j < 3; ++j) {
		C3[j] = (
			po[x] * simplex[(j + 1) % 3][y] +
			po[y] * simplex[(j + 2) % 3][x] +
			simplex[(j + 1) % 3][x] * simplex[(j + 2) % 3][y] -
			po[x] * simplex[(j + 2) % 3][y] -
			po[y] * simplex[(j + 1) % 3][x] -
			simplex[(j + 2) % 3][x] * simplex[(j + 1) % 3][y]
			);
		//C3[j] = (
		//	poXsimpY[(j + 1) % 3] +
		//	poYsimpX[(j + 2) % 3] +
		//	simpXsimpY[(j + 1) % 3][0] -
		//	poXsimpY[(j + 2) % 3] -
		//	poYsimpX[(j + 1) % 3] -
		//	simpXsimpY[(j + 2) % 3][1]
		//	);
	}

	if ((u_max > 0 && glm::all(glm::greaterThan(C3, glm::vec3(0.0f)))) ||
		(u_max < 0 && glm::all(glm::lessThan(C3, glm::vec3(0.0f))))) {
		float tmpr = 1.0f / u_max;
		return { simplex, {C3[0] * tmpr, C3[1] * tmpr, C3[2] * tmpr } };
	}

	float d = std::numeric_limits<float>::max();
	std::vector<glm::vec3> W;
	std::vector<float> Lambda;

	for (int j = 0; j < 3; ++j) {
		if ((u_max >= 0 && -C3[j] >= 0) || (u_max <= 0 && -C3[j] <= 0)) {
			std::vector<glm::vec3> simplex1D = simplex;
			simplex1D.erase(simplex1D.begin() + j);

			auto [W_astrix, Lambda_astrix] = S1D(simplex1D);
			glm::vec3 closestVector = matmul(Lambda_astrix, W_astrix);
			float d_astrix = glm::length(closestVector);
			if (d_astrix < d) {
				W = W_astrix;
				Lambda = Lambda_astrix;
				d = d_astrix;
			}
		}
	}

	return { W, Lambda };
}

std::pair<std::vector<glm::vec3>, std::vector<float>> S3D(const std::vector<Point>& simplex) {
	glm::mat4 M(1.0f);
	for (int i = 0; i < 4; ++i) {
		M[i] = glm::vec4(simplex[i], 1.0f);
	}

	float detM = 0.0f;
	glm::vec4 C4(0.0f);

	// 代数余子式
	for (int j = 0; j < 4; ++j) {
		glm::mat3 minorMat;
		int colIndex = 0;

		for (int col = 0; col < 4; ++col) {
			if (col == j) continue;  // skip the j-th column

			// Copy first 3 rows of this column into minorMat
			minorMat[0][colIndex] = M[0][col];
			minorMat[1][colIndex] = M[1][col];
			minorMat[2][colIndex] = M[2][col];
			colIndex++;
		}

		float sign = ((j + 1 + 4) % 2 == 0) ? 1.0f : -1.0f;
		C4[j] = sign * glm::determinant(minorMat);
		detM += C4[j];
	}

	if ((detM > 0 && glm::all(glm::greaterThan(C4, glm::vec4(0.0f)))) ||
		(detM < 0 && glm::all(glm::lessThan(C4, glm::vec4(0.0f))))) {
		float detMr = 1.0f / detM;
		return { simplex, {C4[0] * detMr, C4[1] * detMr, C4[2] * detMr, C4[3] * detMr} };
	}

	float d = std::numeric_limits<float>::max();
	std::vector<glm::vec3> W;
	std::vector<float> Lambda;

	for (int j = 0; j < 4; ++j) {
		if ((detM >= 0 && -C4[j] >= 0) || (detM <= 0 && -C4[j] <= 0)) {
			std::vector<glm::vec3> simplex2D = simplex;
			simplex2D.erase(simplex2D.begin() + j);

			auto [W_astrix, Lambda_astrix] = S2D(simplex2D);
			glm::vec3 closestVector = matmul(Lambda_astrix, W_astrix);
			float d_astrix = glm::length(closestVector);
			if (d_astrix < d) {
				W = W_astrix;
				Lambda = Lambda_astrix;
				d = d_astrix;
			}
		}
	}

	return { W, Lambda };
}

std::pair<std::vector<glm::vec3>, std::vector<float>> SignedVolumes(std::vector<Point>& simplex)
{
	int size = simplex.size();
	if (size == 4)
	{
		return S3D(simplex);
	}
	else if (size == 3)
	{
		return S2D(simplex);
	}
	else if (size == 2)
	{
		return S1D(simplex);
	}
	return { simplex, {1.0f} };
}


float distanceGJK(const OBB& a, const OBB& b, std::pair<Point, Point>& pointPair)
{
	std::cout << std::fixed << std::setprecision(8);
	//for (int i = 0; i < 8; i++)
	//{
	//	auto p = a.getPoint(i);
	//	std::cout << "[" << p.x << ", " << p.y << ", " << p.z << "]," << std::endl;
	//}
	//for (int i = 0; i < 8; i++)
	//{
	//	auto p = b.getPoint(i);
	//	std::cout << "[" << p.x << ", " << p.y << ", " << p.z << "]," << std::endl;
	//}


	int k = 0;
	Vector dir = a.c - b.c;
	auto [newPoint, dist] = supportMinkowskiDiff(a, b, -dir);

	std::vector<Point> simplex{ newPoint };
	int cnt = 0;
	while (true)
	{
		k++;
		std::pair<std::vector<glm::vec3>, std::vector<float>> result = SignedVolumes(simplex);
		simplex = result.first;
		auto Lambda = result.second;
		dir = matmul(Lambda, simplex);
		//std::cout << "dir : [" << dir.x << ", " << dir.y << ", " << dir.z << "]" << std::endl;
		auto [newPoint, dist] = supportMinkowskiDiff(a, b, -dir);
		float vk_square = glm::dot(dir, dir);
		//std::cout << simplex.size() << std::endl << vk_square << std::endl;
		float gk = vk_square + dist;
		if (gk < TOL || simplex.size() == 4)
		{
			return glm::sqrt(vk_square);
		}
		simplex.insert(simplex.begin(), newPoint);
		//for (const auto& simp : simplex)
		//{
		//	std::cout << "[" << simp.x << ", " << simp.y << ", " << simp.z << "],";
		//}
		//std::cout << std::endl;
		//cnt++;
		//if (cnt > 20)
		//{
		//	exit(0);
		//}
	}
	return 0.0f;
}