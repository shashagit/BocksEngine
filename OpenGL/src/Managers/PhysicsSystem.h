#pragma once

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "../Articulation/Joint.h"
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

	// joints
	std::vector<Joint*> joints;

	void InterpolateState(float);
	void Initialize();

	int impulseIterations;

	bool isResolvingContacts;
	bool isResolvingJoints;
	
	bool applyFriction;
	glm::vec3 gravity;
	float fixedDeltaTime;
	void Update(float);
};

