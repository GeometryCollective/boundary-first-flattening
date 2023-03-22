inline Camera::Camera(float radius_, float theta_, float phi_,
					  glm::vec3 target_, glm::vec3 worldUp_):
radius(radius_),
theta(theta_),
phi(phi_),
target(target_),
worldUp(worldUp_),
viewNeedsUpdate(true),
initialRadius(radius_)
{

}

inline void Camera::reset()
{
	radius = initialRadius;
	theta = 0.0;
	phi = M_PI_2;
	pos = glm::vec3(0.0, 0.0, 0.0);
	target = glm::vec3(0.0, 0.0, 0.0);
	worldUp = glm::vec3(0.0, 1.0, 0.0);
	viewNeedsUpdate = true;
}

inline void Camera::rotate(float dTheta, float dPhi)
{
	if (worldUp.y > 0.0) theta += dTheta;
	else theta -= dTheta;

	phi += dPhi;
	if (phi > 2*M_PI) phi -= 2*M_PI;
	else if (phi < -2*M_PI) phi += 2*M_PI;

	if (dPhi != 0) {
		// if phi is between 0 and pi or -pi and -2pi, up is positive Y, otherwise
		// it is negative Y
		if ((phi > 0 && phi < M_PI) || (phi < -M_PI && phi > -2*M_PI)) worldUp.y = 1.0;
		else worldUp.y = -1.0;
	}

	viewNeedsUpdate = true;
}

inline void Camera::zoom(float distance)
{
   const float minDistance = 0.2;

   if (radius + distance > minDistance) {
	  radius += distance;

   } else {
	   radius = minDistance;
   }

   viewNeedsUpdate = true;
}

inline void Camera::pan(float dx, float dy)
{
	glm::vec3 look = glm::normalize(toCartesian());
	glm::vec3 right = glm::cross(look, worldUp);
	glm::vec3 up = glm::cross(look, right);
	target += right*dx + up*dy;

	viewNeedsUpdate = true;
}

inline glm::vec3 Camera::position()
{
	if (viewNeedsUpdate) {
		pos = target + toCartesian();
	}

	return pos;
}

inline glm::mat4 Camera::orthoMatrix(float left, float right, float bottom, float top,
									 float near, float far) const
{
	return glm::ortho(left, right, bottom, top, near, far);
}

inline glm::mat4 Camera::projectionMatrix(float width, float height,
										  float near, float far) const
{
	return glm::perspective((float)M_PI_4, width/height, near, far);
}

inline glm::mat4 Camera::viewMatrix()
{
	if (viewNeedsUpdate) {
		view = glm::lookAt(position(), target, worldUp);
		viewNeedsUpdate = false;
	}

	return view;
}

inline glm::vec3 Camera::toCartesian() const
{
	float x = radius*sin(phi)*sin(theta);
	float y = radius*cos(phi);
	float z = radius*sin(phi)*cos(theta);

	return glm::vec3(x, y, z);
}
