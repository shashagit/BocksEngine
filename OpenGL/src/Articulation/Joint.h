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

	glm::vec3 impulseSum;
	
	virtual void ApplyImpulse() {}
};

// A Ball and Socket Joint
class BallJoint : public Joint
{
public:
	// these points are in the local space of the body
	glm::vec3 anchorParent, anchorChild;
	Constraint ballJoint;
	float softness, biasFactor;
	
	BallJoint(Body* a, Body* b, glm::vec3 anchor) : Joint(a, b)
	{

		// Tuning
		float frequencyHz = 200.0f;
		float dampingRatio = 0.7f;
		float mass = 50.0f;
		float timeStep = 1.0f / 60.0f;
		
		// frequency in radians
		float omega = 2.0f * 3.14159f * frequencyHz;

		// damping coefficient
		float d = 2.0f * mass * dampingRatio * omega;

		// spring stiffness
		float k = mass * omega * omega;

		// magic formulas
		softness = 1.0f / (d + timeStep * k);
		biasFactor = timeStep * k / (d + timeStep * k);
		
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
		
		glm::vec3 bias = posError * (60.0f) * biasFactor;

		glm::mat3 K = ballJoint.massMatrix.massA - (skewRa * ballJoint.massMatrix.inertiaA * skewRa) +
			ballJoint.massMatrix.massB - (skewRb * ballJoint.massMatrix.inertiaB * skewRb);

		glm::vec3 impulse = glm::inverse(K) * (bias - relativeVelocity - softness * impulseSum);
	
		parent->mVel += impulse * parent->mInvMass;
		parent->mAngularVel += parent->mInertiaWorldInverse * (glm::cross(parent->mRotationMatrix * anchorParent, impulse));

		child->mVel -= impulse * child->mInvMass;
		child->mAngularVel -= child->mInertiaWorldInverse * (glm::cross(child->mRotationMatrix * anchorChild, impulse));

		impulseSum += impulse;
	}
};

// Simplified the hinge joint as a combination of 2 ball joints
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

class RopeJoint : public Joint
{
	float distance;
	Constraint ropeJoint;
public:
	RopeJoint(Body* a, Body* b, float ropeLength) : Joint(a, b), distance((ropeLength))
	{
	}
	void ApplyImpulse() override
	{
		glm::vec3 d = child->mPos - parent->mPos;


		ropeJoint.CalculateMassMatrixInv(parent, child);

		// Calculate Relative Velocity : JV
		glm::vec3 relativeVelocity = child->mVel - parent->mVel;

		// if this relative velocity makes distance lesser next frame then don't
		// need to apply any impulse
		if(glm::length(relativeVelocity * 0.016f + d) < distance)
		{
			return;
		}

		float diff = glm::length(d) - distance;
		// scale the relative velocity by the current distance between the objects
		relativeVelocity *=glm::abs(d);
		
		glm::mat3 K = ropeJoint.massMatrix.massA  +	ropeJoint.massMatrix.massB;
		glm::vec3 bias = -diff * glm::normalize(d) * 0.016f *0.2f;
		glm::vec3 impulse = glm::inverse(K) * ( bias - relativeVelocity);

		parent->mVel -= impulse * parent->mInvMass;
		child->mVel += impulse * child->mInvMass;

		// nullify all angular velocities
		parent->mAngularVel = glm::vec3(0.0f);
		child->mAngularVel = glm::vec3(0.0f);
	}
};