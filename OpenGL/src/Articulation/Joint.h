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
	}

	virtual ~Joint() {}

	Body* parent;
	Body* child;
	
	virtual void ApplyImpulse() {}
};

class BallJoint : public Joint
{
public:
	// these points are in the local space of the body
	glm::vec3 anchorParent, anchorChild;
	Constraint ballJoint;
	
	BallJoint(Body* a, Body* b, glm::vec3 anchor) : Joint(a, b)
	{
		
		anchorParent = glm::transpose(parent->mRotationMatrix) * (anchor - parent->mPos);
		anchorChild = glm::transpose(child->mRotationMatrix) * (anchor - child->mPos);
	}

	void ApplyImpulse() override
	{
		ballJoint.CalculateMassMatrixInv(parent, child);
		//ballJoint.EvaluateVelocityJacobian(parent, child);
		
		glm::vec3 rA = parent->mRotationMatrix * anchorParent;
		glm::vec3 rB = child->mRotationMatrix * anchorChild;

		glm::mat3 skewRa = glm::matrixCross3(rA);
		glm::mat3 skewRb = glm::matrixCross3(rB);

		// Calculate Relative Velocity : JV
		glm::vec3 relativeVelocity = parent->mVel - (skewRa * parent->mAngularVel) -
			child->mVel + (skewRb * child->mAngularVel);
		//std::cout << "relativeVelocity : " << relativeVelocity.x << " : " << relativeVelocity.y << " : " << relativeVelocity.z << std::endl;

		glm::vec3 globalAnchorParent = parent->mRotationMatrix * anchorParent + parent->mPos;
		glm::vec3 globalAnchorChild = child->mRotationMatrix * anchorChild + child->mPos;

		glm::vec3 posError = globalAnchorChild - globalAnchorParent;
		
		glm::vec3 bias = posError * (60.0f) * 0.2f;

		glm::mat3 K = ballJoint.massMatrix.massA - (skewRa * ballJoint.massMatrix.inertiaA * skewRa) +
			ballJoint.massMatrix.massB - (skewRb * ballJoint.massMatrix.inertiaB * skewRb);

		glm::vec3 impulse = glm::inverse(K) * (bias - relativeVelocity);
	
		parent->mVel += impulse * parent->mInvMass;
		parent->mAngularVel += parent->mInertiaWorldInverse * (glm::cross(parent->mRotationMatrix * anchorParent, impulse));

		child->mVel -= impulse * child->mInvMass;
		child->mAngularVel -= child->mInertiaWorldInverse * (glm::cross(child->mRotationMatrix * anchorChild, impulse));
	}
};

class HingeJoint : public Joint
{
public:
	BallJoint bj1, bj2;

	HingeJoint(Body* a, Body* b, glm::vec3 anchor1, glm::vec3 anchor2)
		: Joint(a, b), bj1(BallJoint(a,b, anchor1)), bj2(BallJoint(a,b,anchor2))
	{}

	void ApplyImpulse() override
	{
		bj1.ApplyImpulse();
		bj2.ApplyImpulse();
	}
};