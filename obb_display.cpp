#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>
#include <array>
#include <string_view>

#define WINDOW_WIDTH 2000.f
#define WINDOW_HEIGHT 1500.f
#define DEBUG_DISTANCE
#include "OBB.h"
#include "camera.h" // ������� Camera ��ͷ�ļ�
#include "RectRect.h"


// �ص���������
void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void processInput(GLFWwindow* window);

// ȫ�ֱ���
Camera camera(glm::vec3(0.0f, 0.0f, 10.0f));
float lastX = WINDOW_WIDTH / 2.0;
float lastY = WINDOW_HEIGHT / 2.0;
bool startPressMouse{ true };
bool mousePressed{ false };
bool cameraUpdate{ false };
float deltaTime{ 0.0f };
float lastFrame{ 0.0f };

// ����OBB
void drawOBB(const OBB& obb);

// ��������ϵ
void drawCoordinateSystem();

void drawMinDistSegment(const Point& p1, const Point& p2);

void applyRotate(OBB& obb, const Vector& axis, float angle);

int main() {
	// ��ʼ��GLFW
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_COMPAT_PROFILE);

	// ��������
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

	// ��ʼ��GLAD
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
		std::cout << "Failed to initialize GLAD" << std::endl;
		return -1;
	}

	// ������Ȳ���
	glEnable(GL_DEPTH_TEST);

	OBB a{
		{0, 0, 0},
		{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
		{1, 2, 4}
	};
	OBB b{
		{8, 1, 1 },
		{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
		{ 3, 2, 2}
	};
	applyRotate(a, glm::normalize(Vector{ 1,1,0 }), 20);
	applyRotate(b, glm::normalize(Vector{ 0,1,3 }), 70);

	std::pair<Point, Point> pointPair;
	float dSqr = distance(a, b, pointPair);
	std::cout << "MinDistance: " << sqrt(dSqr) << std::endl;
	std::cout << "Point 1: " << pointPair.first.x << ", " << pointPair.first.y << ", " << pointPair.first.z << std::endl;
	std::cout << "Point 2: " << pointPair.second.x << ", " << pointPair.second.y << ", " << pointPair.second.z << std::endl;

	// ��Ⱦѭ��
	while (!glfwWindowShouldClose(window)) {
		// ����deltaTime
		float currentFrame = static_cast<float>(glfwGetTime());
		deltaTime = currentFrame - lastFrame;
		lastFrame = currentFrame;

		// ���봦��
		processInput(window);

		// ��Ⱦ
		glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// ����ͶӰ����ͼ����
		glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), WINDOW_WIDTH / WINDOW_HEIGHT, 0.1f, 100.0f);
		glm::mat4 view = camera.GetViewMatrix();

		// Ӧ�þ���
		glMatrixMode(GL_PROJECTION);
		glLoadMatrixf(glm::value_ptr(projection));

		glMatrixMode(GL_MODELVIEW);
		glLoadMatrixf(glm::value_ptr(view));

		// ��������ϵ
		drawCoordinateSystem();

		// ����OBB
		drawOBB(a);
		drawOBB(b);

		drawMinDistSegment(pointPair.first, pointPair.second);

		// �����������Ͳ�ѯ�¼�
		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	glfwTerminate();
	return 0;
}

void drawOBB(const OBB& obb) {
	// ��ȡOBB��8������
	std::array<Point, 8> vertices;
	for (unsigned int i = 0; i < 8; ++i) {
		vertices[i] = obb.getPoint(i);
	}

	// ����OBB��12����
	const unsigned int edges[12][2] = {
		{0, 1}, {1, 3}, {3, 2}, {2, 0}, // ����
		{4, 5}, {5, 7}, {7, 6}, {6, 4}, // ����
		{0, 4}, {1, 5}, {2, 6}, {3, 7}  // ����
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

	// X�ᣨ��ɫ��
	glColor3f(1.0f, 0.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(50.0f, 0.0f, 0.0f);

	// Y�ᣨ��ɫ��
	glColor3f(0.0f, 1.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(0.0f, 50.0f, 0.0f);

	// Z�ᣨ��ɫ��
	glColor3f(0.0f, 0.0f, 1.0f);
	glVertex3f(0.0f, 0.0f, 0.0f);
	glVertex3f(0.0f, 0.0f, 50.0f);

	glEnd();
}

void drawMinDistSegment(const Point& p1, const Point& p2) {
	glBegin(GL_LINES);
	glColor3f(1.0f, 1.0f, 0.0f);
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

void applyRotate(OBB& obb, const Vector& axis, float angle)
{
	float rotationAngleRadians = glm::radians(angle); // ת��Ϊ����

	glm::mat4 rotationMatrix = glm::rotate(glm::mat4(1.0f), rotationAngleRadians, axis);

	obb.u[0] = Vector(rotationMatrix * glm::vec4(obb.u[0], 0.0f));
	obb.u[1] = Vector(rotationMatrix * glm::vec4(obb.u[1], 0.0f));
	obb.u[2] = Vector(rotationMatrix * glm::vec4(obb.u[2], 0.0f));
}
