#pragma once

#include <list>
#include "..\Components\Body.h"
#include "..\Components\Collider.h"
#include "../Components/Shape.h"

#define EPSILON 0.000001

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>

#include<algorithm>


struct Contact {
	glm::vec3 point;
	glm::vec3 rA, rB;
	float penetrationDepth;

	float normalImpulseSum; // normal impulses accumulated
	float tangentImpulseSum1; // tangent impulses
	float tangentImpulseSum2;

	Contact() {
		normalImpulseSum = 0.0f;
		tangentImpulseSum1 = 0.0f;
		tangentImpulseSum2 = 0.0f;
	}
};

struct ContactManifold {
	std::vector<Contact*> contactPoints;

	Body *bodyA, *bodyB;

	glm::vec3 collisionNormal;

	ContactManifold()
	{}
	
	ContactManifold(ContactManifold& cm)
	{
		bodyA = cm.bodyA;
		bodyB = cm.bodyB;

		collisionNormal = cm.collisionNormal;
		
		for(auto cp : cm.contactPoints)
		{
			Contact* c = new Contact();
			c = cp;
			contactPoints.push_back(c);
		}
	}
};


class CollisionManager
{
public:
	CollisionManager();
	~CollisionManager();
	
	std::list<ContactManifold*> mContacts;
	std::list<ContactManifold*> mPrevContacts;
	
	void Reset();

};


