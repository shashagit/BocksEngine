#pragma once
#include "../Components/Body.h"
#include "../Resolution/Constraint.h"

#include <glm/gtx/matrix_cross_product.hpp>

class Joint
{
public:
	Joint(Body* a, Body* b)
	{
		parent = a;
		child = b;

		anchorParent = glm::vec3(1.0f, 1.0f, 1.0f); // front top right
		anchorChild = glm::vec3(-1.0f, 1.0f, 1.0f); // front top left
	}

	virtual ~Joint() {}

	Body* parent;
	Body* child;

	// these points are in the local space of the body
	glm::vec3 anchorParent, anchorChild;
	
	virtual glm::vec3 CalculateImpulse() { return glm::vec3(0); }

	void ApplyImpulse(glm::vec3 impulse)
	{
		parent->mVel += impulse * parent->mInvMass;
		child->mVel += -impulse * child->mInvMass;

		parent->mAngularVel += parent->mInertiaWorldInverse * (glm::cross(anchorParent - parent->mPos, impulse));
		child->mAngularVel += child->mInertiaWorldInverse * (glm::cross(anchorChild - child->mPos, -impulse));
	}
};

class BallJoint : public Joint
{
public:
	Constraint ballJoint;
	
	BallJoint(Body* a, Body* b) : Joint(a, b)
	{
		ballJoint.CalculateMassMatrixInv(parent, child);
	}

	glm::vec3 CalculateImpulse() override
	{
		ballJoint.EvaluateVelocityJacobian(parent, child);
		
		glm::vec3 rA = anchorParent * parent->mRotationMatrix + parent->mPos - parent->mPos;
		glm::vec3 rB = anchorChild * child->mRotationMatrix + child->mPos - child->mPos;


		glm::mat3 skewRa = glm::matrixCross3(rA);
		glm::mat3 skewRb = glm::matrixCross3(rB);

		// Calculate Relative Velocity : JV
		glm::vec3 relativeVelocity = parent->mVel - skewRa * parent->mAngularVel -
			child->mVel + skewRb * child->mAngularVel;
		
		glm::mat3 K = ballJoint.massMatrix.massA - skewRa * ballJoint.massMatrix.inertiaA * skewRa +
			ballJoint.massMatrix.massB - skewRb * ballJoint.massMatrix.inertiaB * skewRb;

		glm::mat3 Kinv = glm::inverse(K);

		return Kinv * -relativeVelocity;
	}
};
