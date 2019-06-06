#include "PhysicsSystem.h"

#include "../Resolution/Constraint.h"

#include "../Components/Shape.h"
#include "../Components/Transform.h"
#include "../Components/Collider.h"
#include "../Components/DebugVector.h"
#include "../Managers/CollisionManager.h"
#include "../Managers/FrameRateController.h"
#include "../GameObject/GameObjectManager.h"

#include <list>

extern CollisionManager* colMan;
extern FrameRateController* frc;
extern GameObjectManager* gpGoManager;

PhysicsSystem::PhysicsSystem()
{
	impulseIterations = 8;
	applyFriction = false;
	gravity = glm::vec3(0.0f, -9.8f, 0.0f);
}

void PhysicsSystem::Initialize() {
	dAABBTree.DeleteTree();
	nSquared.RemoveColliders();

	for (auto go : gpGoManager->mGameObjects)
	{
		Collider *pCol = static_cast<Collider*>(go->GetComponent(COLLIDER));
		dAABBTree.AddCollider(pCol);

		//nSquared.AddCollider(pCol);
	}
	
}



void PhysicsSystem::Update(float _deltaTime) {

	fixedDeltaTime = _deltaTime;

	// uncolor all the colliders
	for (auto go : gpGoManager->mGameObjects)
	{
		Collider *pCol = static_cast<Collider*>(go->GetComponent(COLLIDER));
		pCol->isColliding = false;
	}

	//==== check for collisions

	//nSquared.CalculatePairs();
	//std::list < std::pair<Collider*, Collider*>>& pairs = nSquared.GetPairs();

	// update the Dynamic AABB (remove and reinsert)
	dAABBTree.Update();

	// calculate potentially colliding pairs
	dAABBTree.CalculatePairs();
	std::list < std::pair<Collider*, Collider*>>& pairs = dAABBTree.GetPairs();

	// reset the list of Contact manifolds
	colMan->Reset();

	// run Narrow phase on all intersection pairs
	for (auto& pair : pairs) {
		// hack to make the ground not collide with itself
		if (pair.first->mpBody->mMass < 1.9f ||
			pair.second->mpBody->mMass < 1.9f) {

			// perform the SAT intersection test
			if (sat.TestIntersection3D(pair.first, pair.second)) {
				// to color the colliding pair
				pair.first->isColliding = true;
				pair.second->isColliding = true;
			}
		}
	}

	//==== integrate external forces and torques
	for (auto go : gpGoManager->mGameObjects)
	{
		Body *pBody = static_cast<Body*>(go->GetComponent(BODY));
		
		if (pBody->mInvMass == 0.0f)
			continue;

		// hack to make the floor stationary
		if (pBody->mMass >= 1.9f) {
			pBody->mVel = glm::vec3(0.0f, 0.0f, 0.0f);
			pBody->mAngularVel = glm::vec3(0.0f, 0.0f, 0.0f);
			continue;
		}

		// compute acceleration
		glm::vec3 acc = pBody->mTotalForce * pBody->mInvMass;
		acc += gravity;
		glm::vec3 alpha = pBody->mTotalTorque * pBody->mInertiaWorldInverse;

		// integrate acc into the velocity
		pBody->mVel += acc * _deltaTime;
		pBody->mAngularVel += alpha * _deltaTime;

		// set forces to zero
		pBody->mTotalForce = glm::vec3(0);
		pBody->mTotalTorque = glm::vec3(0);
	}

	// small slop
	float slop = -0.005f;
	float mu = 0.1f;

	//==== solve constraints
	for (int i = 0; i < impulseIterations; ++i) {
		for (auto c : colMan->mContacts) {
			
			//std::cout << c->contactPoints[0]->normalImpulseSum << std::endl;
			Constraint constraint;
			constraint.CalculateMassMatrixInv(*c);
			int pointCount = c->contactPoints.size();

			for (int j = 0; j <pointCount; ++j) {

				constraint.EvaluateVelocityVector(*c);

				//===== solve for normal constraint
				constraint.EvaluateJacobian(*c, c->collisionNormal, j);

				float effMass = 1.0f / (constraint.jacobian * constraint.massMatrixInverse * constraint.jacobian.transpose());

				// bias value
				float b = 0.1f / fixedDeltaTime * std::min(c->contactPoints[j]->penetrationDepth - slop, 0.0f);
				//float b = 0.1f / fixedDeltaTime * c->contactPoints[j]->penetrationDepth;

				float lambda = - effMass * (constraint.jacobian * constraint.velocityMatrix + b);
				float origNormalImpulseSum = c->contactPoints[j]->normalImpulseSum;

				c->contactPoints[j]->normalImpulseSum += lambda;
				c->contactPoints[j]->normalImpulseSum =
					glm::clamp(c->contactPoints[j]->normalImpulseSum, 0.0f, std::numeric_limits<float>::infinity());

				float deltaLambda = c->contactPoints[j]->normalImpulseSum - origNormalImpulseSum;

				Eigen::Matrix<float, 12, 1> deltaV =
					constraint.massMatrixInverse * constraint.jacobian.transpose() * deltaLambda;

				c->bodyA->mVel += glm::vec3(deltaV(0, 0), deltaV(1, 0), deltaV(2, 0));
				c->bodyA->mAngularVel += glm::vec3(deltaV(3, 0), deltaV(4, 0), deltaV(5, 0));

				c->bodyB->mVel += glm::vec3(deltaV(6, 0), deltaV(7, 0), deltaV(8, 0));
				c->bodyB->mAngularVel += glm::vec3(deltaV(9, 0), deltaV(10, 0), deltaV(11, 0));

				if (applyFriction) {
					// calculate tangents (Erin Catto's code)
					glm::vec3 t0, t1;

					if (abs(c->collisionNormal.x) >= 0.57735f)
						t0 = glm::normalize(glm::vec3(c->collisionNormal.y, -c->collisionNormal.x, 0.0f));
					else
						t0 = glm::normalize(glm::vec3(0.0f, c->collisionNormal.z, -c->collisionNormal.y));
					t1 = glm::cross(c->collisionNormal, t0);

					//==== solve for tangent 0
					constraint.EvaluateJacobian(*c, t0, j);

					effMass = 1.0f / (constraint.jacobian * constraint.massMatrixInverse * constraint.jacobian.transpose());

					lambda = -effMass * (constraint.jacobian * constraint.velocityMatrix + 0.0f);
					
					float origTangent0ImpulseSum = c->contactPoints[j]->tangentImpulseSum1;

					c->contactPoints[j]->tangentImpulseSum1 += lambda;
					c->contactPoints[j]->tangentImpulseSum1 =
						glm::clamp(c->contactPoints[j]->tangentImpulseSum1, -mu * 9.8f / pointCount, mu * 9.8f / pointCount);

					deltaLambda = c->contactPoints[j]->tangentImpulseSum1 - origTangent0ImpulseSum;

					deltaV = constraint.massMatrixInverse * constraint.jacobian.transpose() * deltaLambda;

					c->bodyA->mVel += glm::vec3(deltaV(0, 0), deltaV(1, 0), deltaV(2, 0));
					c->bodyA->mAngularVel += glm::vec3(deltaV(3, 0), deltaV(4, 0), deltaV(5, 0));

					c->bodyB->mVel += glm::vec3(deltaV(6, 0), deltaV(7, 0), deltaV(8, 0));
					c->bodyB->mAngularVel += glm::vec3(deltaV(9, 0), deltaV(10, 0), deltaV(11, 0));

					//==== solve for tangent 1
					constraint.EvaluateJacobian(*c, t1, j);

					effMass = 1.0f / (constraint.jacobian * constraint.massMatrixInverse * constraint.jacobian.transpose());

					lambda = -effMass * (constraint.jacobian * constraint.velocityMatrix + 0.0f);
					float origTangent1ImpulseSum = c->contactPoints[j]->tangentImpulseSum2;

					c->contactPoints[j]->tangentImpulseSum2 += lambda;
					c->contactPoints[j]->tangentImpulseSum2 =
						glm::clamp(c->contactPoints[j]->tangentImpulseSum2, -mu * 9.8f / pointCount, mu * 9.8f / pointCount);

					deltaLambda = c->contactPoints[j]->tangentImpulseSum2 - origTangent1ImpulseSum;

					deltaV = constraint.massMatrixInverse * constraint.jacobian.transpose() * deltaLambda;

					c->bodyA->mVel += glm::vec3(deltaV(0, 0), deltaV(1, 0), deltaV(2, 0));
					c->bodyA->mAngularVel += glm::vec3(deltaV(3, 0), deltaV(4, 0), deltaV(5, 0));

					c->bodyB->mVel += glm::vec3(deltaV(6, 0), deltaV(7, 0), deltaV(8, 0));
					c->bodyB->mAngularVel += glm::vec3(deltaV(9, 0), deltaV(10, 0), deltaV(11, 0));
				}
			}
		}
	}


	//==== integrate velocity and angular velocity
	for (auto go : gpGoManager->mGameObjects)
	{
		Body *pBody = static_cast<Body*>(go->GetComponent(BODY));
		// save current position
		pBody->mPrevPos = pBody->mPos;

		if (pBody->mInvMass == 0.0f)
			continue;

		// hack to keep the floor stationary
		if (pBody->mMass >= 1.9f) {
			pBody->mVel = glm::vec3(0.0f, 0.0f, 0.0f);
			pBody->mAngularVel = glm::vec3(0.0f, 0.0f, 0.0f);
			continue;
		}
		// integrate the position
		pBody->mPos += pBody->mVel * _deltaTime;
		// integrate the orientation
		glm::fquat newQuat = 0.5f * (pBody->mAngularVel) *pBody->mQuaternion * _deltaTime;
		pBody->mQuaternion *= newQuat;

		// using this instead causes weird behaviour
		//pBody->mQuaternion += 0.5f * glm::fquat(pBody->mAngularVel) *pBody->mQuaternion * _deltaTime;
	}
}

// Interpolates the state of bodies to accurately render them
void PhysicsSystem::InterpolateState(float blendingFactor)
{
	for (auto go : gpGoManager->mGameObjects)
	{
		Body *pBody = static_cast<Body*>(go->GetComponent(BODY));
		Transform* pTr = static_cast<Transform*>(go->GetComponent(TRANSFORM));

		pTr->mPos.x = pBody->mPos.x * blendingFactor + pBody->mPrevPos.x *(1 - blendingFactor);
		pTr->mPos.y = pBody->mPos.y * blendingFactor + pBody->mPrevPos.y *(1 - blendingFactor);
		pTr->mPos.z = pBody->mPos.z * blendingFactor + pBody->mPrevPos.z *(1 - blendingFactor);

		//TODO setup slerp for quaternion interpolation while rendering

		pBody->mQuaternion = glm::normalize(pBody->mQuaternion);
		pBody->mRotationMatrix = glm::toMat3(pBody->mQuaternion);
		pBody->mInertiaWorldInverse = 
			pBody->mRotationMatrix * 
			pBody->mInertiaBodyInverse * 
			glm::transpose(pBody->mRotationMatrix);

		pTr->mRotate = pBody->mRotationMatrix;
	}
}

PhysicsSystem::~PhysicsSystem()
{
}
