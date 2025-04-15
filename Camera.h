#pragma once

enum Camera_Movement
{
	FORWARD,
	BACKWARD,
	LEFT,
	RIGHT,
	UP,
	DOWN
};

const float NEAR_PLANE{ 0.1f };
const float FAR_PLANE{ 100.f };
const float MAX_PITCH_ANGLE{ 89.f };
const float YAW = 0.0f;
const float PITCH = 0.0f;
const float SPEED = 5.0f;
const float SENSITIVITY = 0.2f;
const float ZOOM = 45.0f;

class Camera {
public:
	glm::vec3 Position;
	glm::vec3 Front;
	glm::vec3 Up;
	glm::vec3 Right;
	glm::vec3 WorldUp;

	float NearPlane;
	float FarPlane;
	float Speed;

	float Yaw;
	float Pitch;
	float Zoom;

	Camera(glm::vec3 position = glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f), float nearPlane = NEAR_PLANE, float farPlane = FAR_PLANE, float speed = SPEED, float yaw = YAW, float pitch = PITCH, float zoom = ZOOM) : Front(glm::vec3(0.0f, 0.0f, -1.0f)) {
		Position = position;
		WorldUp = up;
		NearPlane = nearPlane;
		FarPlane = farPlane;
		Speed = speed;
		Yaw = yaw;
		Pitch = pitch;
		Zoom = zoom;
		updateCameraVectors();
	}

	glm::mat4 GetViewMatrix()
	{
		return glm::lookAt(Position, Position + Front, Up);
	}

	glm::mat4 GetProjectionMatrix()
	{
		return glm::perspective(glm::radians(Zoom), 1200.f / 900.f, NearPlane, FarPlane);
	}

	void ProcessKeyboard(Camera_Movement direction, float deltaTime) {
		float velocity = Speed * deltaTime;
		if (direction == FORWARD)
			Position += Front * velocity;
		if (direction == BACKWARD)
			Position -= Front * velocity;
		if (direction == LEFT)
			Position -= Right * velocity;
		if (direction == RIGHT)
			Position += Right * velocity;
		if (direction == UP)
			Position += Up * velocity;
		if (direction == DOWN)
			Position -= Up * velocity;
	}

	void ProcessMouseMovement(float xoffset, float yoffset, bool constrainPitch = true) {
		xoffset *= SENSITIVITY;
		yoffset *= SENSITIVITY;

		Yaw += xoffset;
		Pitch += yoffset;

		if (constrainPitch) {
			if (Pitch > MAX_PITCH_ANGLE)
				Pitch = MAX_PITCH_ANGLE;
			if (Pitch < -MAX_PITCH_ANGLE)
				Pitch = -MAX_PITCH_ANGLE;
		}

		updateCameraVectors();
	}

private:
	void updateCameraVectors() {
		glm::vec3 front;
		front.x = cos(glm::radians(Pitch)) * sin(glm::radians(Yaw));
		front.y = sin(glm::radians(Pitch));
		front.z = -cos(glm::radians(Pitch)) * cos(glm::radians(Yaw));
		Front = glm::normalize(front);
		Right = glm::normalize(glm::cross(Front, WorldUp));
		Up = glm::normalize(glm::cross(Right, Front));
	}
};