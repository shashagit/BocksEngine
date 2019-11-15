#pragma once
#include "../Components/Body.h"
#include <glm/gtx/matrix_cross_product.hpp>

class Joint
{
public:
	Body* parent;
	Body* child;

	glm::vec3 anchorParent, anchorChild;
	
	void ApplyImpulse(glm::vec3 impulse)
	{
		parent->mVel += impulse * parent->mInvMass;
		child->mVel += -impulse * child->mInvMass;

		parent->mAngularVel += parent->mInertiaWorldInverse * (glm::cross(anchorParent - parent->mPos, impulse));
		child->mAngularVel += child->mInertiaWorldInverse * (glm::cross(anchorChild - child->mPos, impulse));
	}
};

class BallJoint : public Joint
{
	
};
