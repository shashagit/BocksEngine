#pragma once
#include "glm/vec3.hpp"
#include "../Resolution/PhysicsDataTypes.h"

struct Contact {
	glm::vec3 point;
	glm::vec3 rA, rB;
	float penetrationDepth;

	float normalImpulseSum; // normal impulses accumulated
	float tangentImpulseSum1; // tangent impulses
	float tangentImpulseSum2;

	Jacobian jacobianN, jacobianT0, jacobianT1;

	float effectiveMassN, effectiveMassT0, effectiveMassT1;
	VelocityJacobian  mMatxjN, mMatxjT0, mMatxjT1;

	Contact() {
		point = glm::vec3(0);
		normalImpulseSum = 0.0f;
		tangentImpulseSum1 = 0.0f;
		tangentImpulseSum2 = 0.0f;
	}
};
