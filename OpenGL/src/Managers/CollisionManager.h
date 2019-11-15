#pragma once

#include <list>
#include "..\Components\Body.h"
#include "..\Components\Collider.h"
#include "../Components/Shape.h"

#define EPSILON 0.000001

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>

#include<algorithm>
#include "../Resolution/Constraint.h"
#include "../NarrowPhase/Contact.h"


class ContactManifold {
public:
	std::vector<Contact> contactPoints;

	Body* bodyA, * bodyB;
	Constraint constraint;

	glm::vec3 collisionNormal;
	glm::vec3 t0, t1;

	ContactManifold() : collisionNormal(0), t0(0), t1(0)
	{}
	//EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	void SetupGroundConstraint()
	{
		// calculate tangents (Erin Catto's code)
		if (abs(collisionNormal.x) >= 0.57735f)
			t0 = glm::normalize(glm::vec3(collisionNormal.y, -collisionNormal.x, 0.0f));
		else
			t0 = glm::normalize(glm::vec3(0.0f, collisionNormal.z, -collisionNormal.y));
		t1 = glm::cross(collisionNormal, t0);

		constraint.CalculateMassMatrixInv(bodyA, bodyB);

		for (auto& c : contactPoints)
		{
			constraint.EvaluateJacobian(c.jacobianN, &c, collisionNormal);
			constraint.CalculateEffectiveMass(c.mMatxjN, c.jacobianN, c.effectiveMassN);
			constraint.EvaluateJacobian(c.jacobianT0, &c, t0);
			constraint.CalculateEffectiveMass(c.mMatxjT0, c.jacobianT0, c.effectiveMassT0);
			constraint.EvaluateJacobian(c.jacobianT1, &c, t1);
			constraint.CalculateEffectiveMass(c.mMatxjT1, c.jacobianT1, c.effectiveMassT1);
		}
	}

	ContactManifold(ContactManifold& cm)
	{
		bodyA = cm.bodyA;
		bodyB = cm.bodyB;

		collisionNormal = cm.collisionNormal;
		constraint = cm.constraint;

		for (auto cp : cm.contactPoints)
		{
			Contact c(cp);
			contactPoints.emplace_back(c);
		}
	}
};


class CollisionManager
{
public:
	CollisionManager();
	~CollisionManager();
	
	std::list<ContactManifold*>* mContacts;
	std::list<ContactManifold*>* mPrevContacts;
	
	void Reset();

};


