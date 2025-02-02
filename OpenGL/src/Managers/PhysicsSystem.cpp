#include "PhysicsSystem.h"

#include "../Resolution/Constraint.h"

#include "../Components/Shape.h"
#include "../Components/Transform.h"
#include "../Components/Collider.h"
#include "../Components/DebugVector.h"
#include "../Managers/CollisionManager.h"
#include "../Managers/FrameRateController.h"
#include "../GameObject/GameObjectManager.h"

#include "imgui/imgui.h"

#include <list>
#include "../GameObject/Profiler.h"

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
		Collider* pCol = static_cast<Collider*>(go->GetComponent(COLLIDER));
		pCol->isColliding = false;
	}

	//==== check for collisions
	//std::cout << "===================" << std::endl;

	//nSquared.CalculatePairs();
	//std::list < std::pair<Collider*, Collider*>>& pairs = nSquared.GetPairs();
	{
		//std::cout << "Broadphase ";
		//Timer t;
		// update the Dynamic AABB (remove and reinsert)
		dAABBTree.Update();

		// calculate potentially colliding pairs
		dAABBTree.CalculatePairs();

	}
	std::list < std::pair<Collider*, Collider*>>& pairs = dAABBTree.GetPairs();
	// reset the list of Contact manifolds
	colMan->Reset();

	{
		//std::cout << "SAT ";
		//Timer t;
		// run Narrow phase on all intersection pairs
		for (auto& pair : pairs) {
			// hack to make the ground not collide with itself
			
				// perform the SAT intersection test
				if (sat.TestIntersection3D(pair.first, pair.second)) {
					// to color the colliding pair
					pair.first->isColliding = true;
					pair.second->isColliding = true;
				}
			
		}
	}

	//==== integrate external forces and torques
	{
		//std::cout << "Force Integration ";
		//Timer t;

		for (auto go : gpGoManager->mGameObjects)
		{
			Body* pBody = static_cast<Body*>(go->GetComponent(BODY));

			if (pBody->mInvMass == 0.0f)
				continue;

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
	}


	// small slop
	float slop = -0.005f;
	float mu = 0.2f;
	float baumgarte = 0.1f;
	const float bias = 1.0f;
	const float proximityEpsilon = 0.00001f;
	
	bool isWarmStarting = true;
	//==== warm starting
	{
		//std::cout << "Warm Start ";
		//Timer t;

		if (isWarmStarting) {
			if (!colMan->mPrevContacts->empty())
			{
				for (auto itOld : *colMan->mPrevContacts)
				{
					for (auto itNew : *colMan->mContacts)
					{
						if (((*itNew).bodyA == (*itOld).bodyA &&
							(*itNew).bodyB == (*itOld).bodyB)) {

							// iterate through contact points
							for (int j = 0; j < (*itOld).contactPoints.size(); ++j)
							{
								Contact& cOld = (*itOld).contactPoints[j];

								for (int k = 0; k < (*itNew).contactPoints.size(); ++k) {
									Contact& cNew = (*itNew).contactPoints[k];

									if (glm::distance2(cNew.point, cOld.point) < proximityEpsilon)
									{
										cNew.normalImpulseSum = cOld.normalImpulseSum * bias;
										cNew.tangentImpulseSum1 = cOld.tangentImpulseSum1 * bias;
										cNew.tangentImpulseSum2 = cOld.tangentImpulseSum2 * bias;

										// apply old impulse as warm start
										(itNew)->constraint.EvaluateJacobian(&cNew, itNew->collisionNormal);
										itNew->constraint.ApplyImpulse(itNew->bodyA, itNew->bodyB, cNew.normalImpulseSum);

										itNew->constraint.EvaluateJacobian(&cNew, itNew->t0);
										itNew->constraint.ApplyImpulse(itNew->bodyA, itNew->bodyB,  cNew.tangentImpulseSum1);
										
										itNew->constraint.EvaluateJacobian(&cNew, itNew->t1);
										itNew->constraint.ApplyImpulse(itNew->bodyA, itNew->bodyB, cNew.tangentImpulseSum2);
									}
								}

							}

						}
					}
				}
			}
		}
	}

	/*if (!colMan->mContacts->empty()) {
		for (auto c : *colMan->mContacts) {
		ImGui::Begin("Contact Manifold");
			ImGui::PushID(c);
			ImGui::Text("penetration depth %f", (*c).contactPoints[0].penetrationDepth);
			ImGui::Text("normal impulse sum %f", (*c).contactPoints[0].normalImpulseSum);
			ImGui::Text("t0 impulse sum %f", (*c).contactPoints[0].tangentImpulseSum1);
			ImGui::Text("t1 impulse sum %f", (*c).contactPoints[0].tangentImpulseSum2);
			ImGui::PopID();
		ImGui::End();
		}
	}*/

	{
		//std::cout << "SI solver ";
		//Timer t;
		//==== solve constraints
		for (int i = 0; i < impulseIterations; ++i) {
			for (auto c : *colMan->mContacts) {

				//std::cout << c->contactPoints[0]->normalImpulseSum << std::endl;
				int pointCount = c->contactPoints.size();

				for (int j = 0; j < pointCount; ++j) {

					c->constraint.EvaluateVelocityVector(c->bodyA, c->bodyB);

					//===== solve for normal constraint
					c->constraint.EvaluateJacobian(&c->contactPoints[j], c->collisionNormal);

					float effMass = 1.0f / (c->constraint.jacobian * c->constraint.massMatrixInverse * c->constraint.jacobianT);

					// bias value
					float b = baumgarte / fixedDeltaTime * std::min(c->contactPoints[j].penetrationDepth - slop, 0.0f);
					//float b = 0.1f / fixedDeltaTime * c->contactPoints[j]->penetrationDepth;

					float lambda = -effMass * (c->constraint.jacobian * c->constraint.velocityMatrix + b);
					float origNormalImpulseSum = c->contactPoints[j].normalImpulseSum;

					c->contactPoints[j].normalImpulseSum += lambda;
					c->contactPoints[j].normalImpulseSum =
						glm::clamp(c->contactPoints[j].normalImpulseSum, 0.0f, std::numeric_limits<float>::infinity());

					float deltaLambda = c->contactPoints[j].normalImpulseSum - origNormalImpulseSum;

					c->constraint.ApplyImpulse(c->bodyA, c->bodyB, deltaLambda);

					if (applyFriction) {
						//float nLambda = c->contactPoints[j]->normalImpulseSum;
						float nLambda = -gravity.y / pointCount;

						// calculate tangents (Erin Catto's code)
						glm::vec3 t0, t1;

						if (abs(c->collisionNormal.x) >= 0.57735f)
							t0 = glm::normalize(glm::vec3(c->collisionNormal.y, -c->collisionNormal.x, 0.0f));
						else
							t0 = glm::normalize(glm::vec3(0.0f, c->collisionNormal.z, -c->collisionNormal.y));
						t1 = glm::cross(c->collisionNormal, t0);

						//==== solve for tangent 0
						c->constraint.EvaluateJacobian(&c->contactPoints[j], t0);

						effMass = 1.0f / (c->constraint.jacobian * c->constraint.massMatrixInverse * c->constraint.jacobianT);

						lambda = -effMass * (c->constraint.jacobian * c->constraint.velocityMatrix + 0.0f);

						float origTangent0ImpulseSum = c->contactPoints[j].tangentImpulseSum1;

						c->contactPoints[j].tangentImpulseSum1 += lambda;
						c->contactPoints[j].tangentImpulseSum1 =
							glm::clamp(c->contactPoints[j].tangentImpulseSum1, -mu * nLambda, mu * nLambda);

						deltaLambda = c->contactPoints[j].tangentImpulseSum1 - origTangent0ImpulseSum;

						c->constraint.ApplyImpulse(c->bodyA, c->bodyB, deltaLambda);

						//==== solve for tangent 1
						c->constraint.EvaluateJacobian(&c->contactPoints[j], t1);

						effMass = 1.0f / (c->constraint.jacobian * c->constraint.massMatrixInverse * c->constraint.jacobianT);

						lambda = -effMass * (c->constraint.jacobian * c->constraint.velocityMatrix + 0.0f);
						float origTangent1ImpulseSum = c->contactPoints[j].tangentImpulseSum2;

						c->contactPoints[j].tangentImpulseSum2 += lambda;
						c->contactPoints[j].tangentImpulseSum2 =
							glm::clamp(c->contactPoints[j].tangentImpulseSum2, -mu * nLambda, mu * nLambda);

						deltaLambda = c->contactPoints[j].tangentImpulseSum2 - origTangent1ImpulseSum;

						c->constraint.ApplyImpulse(c->bodyA, c->bodyB, deltaLambda);
					}
				}
			}
		}
	}

	{
		//std::cout << "Copy ";
		//Timer t;
		// Copy contacts into previous list
		colMan->mPrevContacts = colMan->mContacts;
	}
	
	{
		//std::cout << "Velocity integration ";
		//Timer t;
		//==== integrate velocity and angular velocity
		for (auto go : gpGoManager->mGameObjects)
		{
			Body* pBody = static_cast<Body*>(go->GetComponent(BODY));

			// save current position
			pBody->mPrevPos = pBody->mPos;

			// integrate the position
			pBody->mPos += pBody->mVel * _deltaTime;
			// integrate the orientation
			glm::fquat newQuat = 0.5f * (pBody->mAngularVel) * pBody->mQuaternion * _deltaTime;
			pBody->mQuaternion *= newQuat;

			// using this instead causes weird behaviour
			//pBody->mQuaternion += 0.5f * glm::fquat(pBody->mAngularVel) *pBody->mQuaternion * _deltaTime;
		}
	}

	//std::cout << "===================" << std::endl;
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

		pTr->mRotate = glm::toMat4(pBody->mQuaternion);
	}
}

PhysicsSystem::~PhysicsSystem()
{
}
