#pragma once

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "../Broadphase/NSquaredBroadphase.h"
#include "../Broadphase/DynamicAABBTree.h"
#include "../NarrowPhase/SAT.h"

class PhysicsSystem
{
public:
	PhysicsSystem();
	~PhysicsSystem();
	
	// broadphase
	NSquaredBroadphase nSquared;
	DynamicAABBTree dAABBTree;

	// narrowphase
	SAT sat;

	void InterpolateState(float);
	void Initialize();

	int impulseIterations;

	bool applyFriction;
	glm::vec3 gravity;
	float fixedDeltaTime;
	void Update(float);
};

