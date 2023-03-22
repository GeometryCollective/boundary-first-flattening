#pragma once

#define _USE_MATH_DEFINES

#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "math.h"

class Camera {
public:
	// constructor
	Camera(float radius, float theta = 0.0, float phi = M_PI_2,
		   glm::vec3 target = glm::vec3(0.0, 0.0, 0.0),
		   glm::vec3 worldUp = glm::vec3(0.0, 1.0, 0.0));

	// restores default setting
	void reset();

	// rotates camera about target
	void rotate(float dTheta, float dPhi);

	// moves camera along the look vector
	void zoom(float distance);

	// moves camera within its local xy plane
	void pan(float dx, float dy);

	// returns camera position
	glm::vec3 position();

	// returns ortho matrix
	glm::mat4 orthoMatrix(float left, float right, float bottom, float top,
						  float near = 0.01, float far = 1000.0) const;

	// returns projection matrix
	glm::mat4 projectionMatrix(float width, float height,
							   float near = 0.1, float far = 1000.0) const;

	// returns view matrix
	glm::mat4 viewMatrix();

	// members
	float radius;
	float theta;
	float phi;
	glm::vec3 pos;
	glm::vec3 target;
	glm::vec3 worldUp;
	bool viewNeedsUpdate;

private:
	// converts spherical coordinates to cartesian coordinates
	glm::vec3 toCartesian() const;

	// members
	float initialRadius;
	glm::mat4 view;
};

#include "Camera.inl"
