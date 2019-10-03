#pragma once
#include "glm/vec3.hpp"

struct Contact {
	glm::vec3 point;
	glm::vec3 rA, rB;
	float penetrationDepth;

	float normalImpulseSum; // normal impulses accumulated
	float tangentImpulseSum1; // tangent impulses
	float tangentImpulseSum2;

	Contact() {
		point = glm::vec3(0);
		normalImpulseSum = 0.0f;
		tangentImpulseSum1 = 0.0f;
		tangentImpulseSum2 = 0.0f;
	}
};
