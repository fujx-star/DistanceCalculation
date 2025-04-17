#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>
#include <array>
#include <string_view>

#define WINDOW_WIDTH 1500.f
#define WINDOW_HEIGHT 1000.f
#define WINDOW_ASPECT (WINDOW_WIDTH / WINDOW_HEIGHT)
#define DEBUG_DISTANCEx
#include "OBB.h"
#include "camera.h" // 包含你的 Camera 类头文件
#include "RectRect.h"
#include "SAT.h"
#include "sample.h"


// 回调函数声明
void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void processInput(GLFWwindow* window);

// 全局变量
Camera camera(glm::vec3(0.0f, 0.0f, 10.0f));
float lastX = WINDOW_WIDTH / 2.0;
float lastY = WINDOW_HEIGHT / 2.0;
bool startPressMouse{ true };
bool mousePressed{ false };
bool cameraUpdate{ false };
float deltaTime{ 0.0f };
float lastFrame{ 0.0f };

Vector rotation;
Vector translation;
bool pauseOutput = false;
bool keyPressed = false;
Vector origin{ 8, 4, 4 };

// 绘制OBB
void drawOBB(const OBB& obb);

// 绘制坐标系
void drawCoordinateSystem();

void drawMinDistSegment(const Point& p1, const Point& p2, const Vector& lineColor);

void applyRotate(OBB& obb);
void applyTranslate(OBB& obb);

int main() {
	// 初始化GLFW
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);

	// 创建窗口
	GLFWwindow* window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "OBB with Coordinate System", NULL, NULL);
	if (window == NULL) {
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return -1;
	}
	glfwMakeContextCurrent(window);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
	glfwSetCursorPosCallback(window, mouse_callback);
	glfwSetScrollCallback(window, scroll_callback);

	// 初始化GLAD
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
		std::cout << "Failed to initialize GLAD" << std::endl;
		return -1;
	}

	// 启用深度测试
	glEnable(GL_DEPTH_TEST);

	OBB a{
		{0, 0, 0},
		{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
		{1, 2, 4}
	};
	OBB b{
		origin,
		{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
		{1, 1, 1}
	};


	//std::pair<Point, Point> pointPair;
	//float dk = distanceSample(a, b, pointPair);
	//std::cout << "MinDistance Sample: " << dk << std::endl;
	//std::cout << "Point 1: " << pointPair.first.x << ", " << pointPair.first.y << ", " << pointPair.first.z << std::endl;
	//std::cout << "Point 2: " << pointPair.second.x << ", " << pointPair.second.y << ", " << pointPair.second.z << std::endl;
	std::pair<Point, Point> pointPair0, pointPair1;

	// 渲染循环
	while (!glfwWindowShouldClose(window)) {
		// 计算deltaTime
		float currentFrame = static_cast<float>(glfwGetTime());
		deltaTime = currentFrame - lastFrame;
		lastFrame = currentFrame;

		// 输入处理
		processInput(window);
		applyRotate(b);
		applyTranslate(b);

		// 渲染
		glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// 设置投影和视图矩阵
		//glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), WINDOW_WIDTH / WINDOW_HEIGHT, 0.1f, 100.0f);
		float size{ 10 };
		glm::mat4 projection = glm::ortho(-size * WINDOW_ASPECT, size * WINDOW_ASPECT, -size, size, 0.1f, 10000.0f);
		glm::mat4 view = camera.GetViewMatrix();

		// 应用矩阵
		glMatrixMode(GL_PROJECTION);
		glLoadMatrixf(glm::value_ptr(projection));

		glMatrixMode(GL_MODELVIEW);
		glLoadMatrixf(glm::value_ptr(view));

		// 绘制坐标系
		drawCoordinateSystem();

		// 绘制OBB
		drawOBB(a);
		drawOBB(b);

		float d0 = sqrt(distanceRectRect(a, b, pointPair0));
		float d1 = sqrt(distanceSAT(a, b, pointPair1));
		if (NE(d0, d1)) {
			std::cerr << "Assertion failed: distanceRectRect (" << d0 << ") != distanceSAT (" << d1 << ")" << std::endl;
			//assert(EQ(d0, d1)); // 这里的断言只起到终止程序的作用
		}

		if (!pauseOutput)
		{
			std::cout << "MinDistance RectRect: " << d0 << std::endl;
			std::cout << "Point 1: " << pointPair0.first.x << ", " << pointPair0.first.y << ", " << pointPair0.first.z << std::endl;
			std::cout << "Point 2: " << pointPair0.second.x << ", " << pointPair0.second.y << ", " << pointPair0.second.z << std::endl;
			std::cout << "MinDistance SAT: " << d1 << std::endl;
			std::cout << "Point 1: " << pointPair1.first.x << ", " << pointPair1.first.y << ", " << pointPair1.first.z << std::endl;
			std::cout << "Point 2: " << pointPair1.second.x << ", " << pointPair1.second.y << ", " << pointPair1.second.z << std::endl;
		}

		drawMinDistSegment(pointPair0.first, pointPair0.second, { 1.0f, 1.0f, 0.0f });
		drawMinDistSegment(pointPair1.first, pointPair1.second, { 0.0f, 1.0f, 1.0f });
		//drawMinDistSegment(pointPair.first, pointPair.second, { 0.0f, 1.0f, 0.0f });

		// 交换缓冲区和查询事件
		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	glfwTerminate();
	return 0;
}

void drawOBB(const OBB& obb) {
	// 获取OBB的8个顶点
	std::array<Point, 8> vertices;
	for (unsigned int i = 0; i < 8; ++i) {
		vertices[i] = obb.getPoint(i);
	}

	//// 定义OBB的12条边
	const unsigned int edges[12][2] = {
		{0, 1}, {1, 3}, {3, 2}, {2, 0}, // 底面
		{4, 5}, {5, 7}, {7, 6}, {6, 4}, // 顶面
		{0, 4}, {1, 5}, {2, 6}, {3, 7}  // 侧面
	};

	glBegin(GL_LINES);
	glColor3f(1.0f, 1.0f, 1.0f);
	for (const auto& edge : edges) {
		glVertex3f(vertices[edge[0]].x, vertices[edge[0]].y, vertices[edge[0]].z);
		glVertex3f(vertices[edge[1]].x, vertices[edge[1]].y, vertices[edge[1]].z);
	}
	glEnd();
}

void drawCoordinateSystem() {
	glBegin(GL_LINES);

	// X轴（红色）
	glColor3f(1.0f, 0.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(50.0f, 0.0f, 0.0f);

	// Y轴（绿色）
	glColor3f(0.0f, 1.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(0.0f, 50.0f, 0.0f);

	// Z轴（蓝色）
	glColor3f(0.0f, 0.0f, 1.0f);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, 50.0f);

	glEnd();
}

void drawMinDistSegment(const Point& p1, const Point& p2, const Vector& lineColor) {
	glBegin(GL_LINES);
	glColor3f(lineColor.x, lineColor.y, lineColor.z);
	glVertex3f(p1.x, p1.y, p1.z);
	glVertex3f(p2.x, p2.y, p2.z);
	glEnd();
}

void processInput(GLFWwindow* window) {
	float cameraSpeed = camera.Speed * deltaTime;
	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
		camera.ProcessKeyboard(FORWARD, deltaTime);
	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
		camera.ProcessKeyboard(BACKWARD, deltaTime);
	if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
		camera.ProcessKeyboard(LEFT, deltaTime);
	if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
		camera.ProcessKeyboard(RIGHT, deltaTime);
	if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS)
		camera.ProcessKeyboard(UP, deltaTime);
	if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS)
		camera.ProcessKeyboard(DOWN, deltaTime);

	if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) {
		if (!keyPressed) {
			pauseOutput = !pauseOutput; 
			keyPressed = true;
		}
	}
	else if (glfwGetKey(window, GLFW_KEY_E) == GLFW_RELEASE) {
		keyPressed = false;
	}

	if (glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS)
		rotation.x += cameraSpeed;
	if (glfwGetKey(window, GLFW_KEY_T) == GLFW_PRESS)
		rotation.x -= cameraSpeed;
	if (glfwGetKey(window, GLFW_KEY_Y) == GLFW_PRESS)
		rotation.y += cameraSpeed;
	if (glfwGetKey(window, GLFW_KEY_U) == GLFW_PRESS)
		rotation.y -= cameraSpeed;
	if (glfwGetKey(window, GLFW_KEY_I) == GLFW_PRESS)
		rotation.z += cameraSpeed;
	if (glfwGetKey(window, GLFW_KEY_O) == GLFW_PRESS)
		rotation.z -= cameraSpeed;

	if (glfwGetKey(window, GLFW_KEY_F) == GLFW_PRESS)
		translation.x += cameraSpeed;
	if (glfwGetKey(window, GLFW_KEY_G) == GLFW_PRESS)
		translation.x -= cameraSpeed;
	if (glfwGetKey(window, GLFW_KEY_H) == GLFW_PRESS)
		translation.y += cameraSpeed;
	if (glfwGetKey(window, GLFW_KEY_J) == GLFW_PRESS)
		translation.y -= cameraSpeed;
	if (glfwGetKey(window, GLFW_KEY_K) == GLFW_PRESS)
		translation.z += cameraSpeed;
	if (glfwGetKey(window, GLFW_KEY_L) == GLFW_PRESS)
		translation.z -= cameraSpeed;
}

void mouse_callback(GLFWwindow* window, double xpos, double ypos)
{
	if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS)
	{
		mousePressed = true;
		if (startPressMouse)
		{
			lastX = static_cast<float>(xpos);
			lastY = static_cast<float>(ypos);
			startPressMouse = false;
		}
	}
	else if (mousePressed && glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_RELEASE)
	{
		mousePressed = false;
		startPressMouse = true;
	}
	if (mousePressed)
	{
		float xoffset = static_cast<float>(xpos) - lastX;
		float yoffset = static_cast<float>(ypos) - lastY;

		lastX = static_cast<float>(xpos);
		lastY = static_cast<float>(ypos);

		// origin of vulkan screen coordinate is at the left top corner
		camera.ProcessMouseMovement(xoffset, -yoffset);
	}
}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
	camera.Zoom -= (float)yoffset;
	if (camera.Zoom < 1.0f)
		camera.Zoom = 1.0f;
	if (camera.Zoom > 45.0f)
		camera.Zoom = 45.0f;
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
	glViewport(0, 0, width, height);
}

void applyRotate(OBB& obb)
{
	glm::mat4 rotationMatrix = glm::mat4(1.0f); // 单位矩阵
	rotationMatrix = glm::rotate(rotationMatrix, glm::radians(rotation.x), glm::vec3(1.0f, 0.0f, 0.0f)); // 绕X轴旋转
	rotationMatrix = glm::rotate(rotationMatrix, glm::radians(rotation.y), glm::vec3(0.0f, 1.0f, 0.0f)); // 绕Y轴旋转
	rotationMatrix = glm::rotate(rotationMatrix, glm::radians(rotation.z), glm::vec3(0.0f, 0.0f, 1.0f)); // 绕Z轴旋转

	//obb.u[0] = Vector(rotationMatrix * glm::vec4(obb.u[0], 0.0f));
	//obb.u[1] = Vector(rotationMatrix * glm::vec4(obb.u[1], 0.0f));
	//obb.u[2] = Vector(rotationMatrix * glm::vec4(obb.u[2], 0.0f));
	obb.u[0] = Vector(rotationMatrix * glm::vec4(1.0f, 0.0f, 0.0f, 0.0f));
	obb.u[1] = Vector(rotationMatrix * glm::vec4(0.0f, 1.0f, 0.0f, 0.0f));
	obb.u[2] = Vector(rotationMatrix * glm::vec4(0.0f, 0.0f, 1.0f, 0.0f));
}

void applyTranslate(OBB& obb)
{
	obb.c = translation + origin;
}