#pragma once
#include <glm/mat3x3.hpp>

struct VelocityJacobian
{
	glm::vec3 velocityA;
	glm::vec3 angularVelocityA;
	glm::vec3 velocityB;
	glm::vec3 angularVelocityB;
};

struct Jacobian
{
	glm::vec3 positionA;
	glm::vec3 orientationA;
	glm::vec3 positionB;
	glm::vec3 orientationB;
};

struct MassMatrix
{
	glm::mat3 massA;
	glm::mat3 inertiaA;
	glm::mat3 massB;
	glm::mat3 inertiaB;
};