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

		anchorParent = glm::vec3(0.5f, 0.5f, 0.5f); // front top right
		anchorChild = glm::vec3(-0.5f, 0.5f, 0.5f); // front top left
	}

	virtual ~Joint() {}

	Body* parent;
	Body* child;

	// these points are in the local space of the body
	glm::vec3 anchorParent, anchorChild;
	
	virtual glm::vec3 CalculateImpulse() { return glm::vec3(0); }

	void ApplyImpulse(glm::vec3 impulse)
	{
		impulse *= 0.8;

		parent->mVel += impulse * parent->mInvMass;
		child->mVel += -impulse * child->mInvMass;

		glm::vec3 rA = parent->mRotationMatrix * anchorParent;
		glm::vec3 rB = child->mRotationMatrix * anchorChild;

		parent->mAngularVel += parent->mInertiaWorldInverse * (glm::cross(rA, impulse));
		child->mAngularVel += child->mInertiaWorldInverse * (glm::cross(rB, -impulse));
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
		
		glm::vec3 rA = parent->mRotationMatrix * anchorParent;
		glm::vec3 rB = child->mRotationMatrix * anchorChild;

		glm::mat3 skewRa = glm::matrixCross3(rA);
		glm::mat3 skewRb = glm::matrixCross3(rB);

		// Calculate Relative Velocity
		glm::vec3 relativeVelocity = parent->mVel - skewRa * parent->mAngularVel -
			child->mVel + skewRb * child->mAngularVel;
		
		std::cout << "relativeVelocity : " << relativeVelocity.x << " : " << relativeVelocity.y << " : " << relativeVelocity.z << std::endl;

		glm::mat3 K = ballJoint.massMatrix.massB - skewRb * ballJoint.massMatrix.inertiaB * skewRb +
			ballJoint.massMatrix.massA - skewRa * ballJoint.massMatrix.inertiaA * skewRa;

		glm::mat3 Kinv = glm::inverse(K);

		return Kinv * -relativeVelocity;
	}
};
