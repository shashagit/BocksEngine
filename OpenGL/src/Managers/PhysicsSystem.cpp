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

float angularDamp = 0.05f;
float linearDamp = 0.005f;

PhysicsSystem::PhysicsSystem()
{
	impulseIterations = 8;
	applyFriction = false;
	isResolvingContacts = true;
	isResolvingJoints = false;
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



void PhysicsSystem::Update(float _deltaTime)
{

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

			// damping velocities
			pBody->mAngularVel *= 1.0f / (1.0f + _deltaTime * angularDamp);
			pBody->mVel *= 1.0f / (1.0f + _deltaTime * linearDamp);
		}
	}


	// small slop
	float slop = -0.005f;
	float mu = 0.02f;
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
										itNew->constraint.ApplyImpulse(itNew->bodyA, itNew->bodyB, cNew.mMatxjN, cNew.normalImpulseSum);
										itNew->constraint.ApplyImpulse(itNew->bodyA, itNew->bodyB, cNew.mMatxjT0, cNew.tangentImpulseSum1);
										itNew->constraint.ApplyImpulse(itNew->bodyA, itNew->bodyB, cNew.mMatxjT1, cNew.tangentImpulseSum2);
									}
								}

							}

						}
					}
				}
			}
		}
	}

	for (auto j : joints)
	{
		j->impulseSum = glm::vec3(0.0f);
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


	//std::cout << "SI solver ";
	//Timer t;
	//==== solve constraints
	for (int i = 0; i < impulseIterations; ++i) {
		if (isResolvingContacts)
		{
			for (auto c : *colMan->mContacts) {

				//std::cout << c->contactPoints[0]->normalImpulseSum << std::endl;
				int pointCount = c->contactPoints.size();

				for (int j = 0; j < pointCount; ++j) {

					c->constraint.EvaluateVelocityJacobian(c->bodyA, c->bodyB);

					//===== solve for normal constraint
					// bias value
					float b = baumgarte / fixedDeltaTime * std::min(c->contactPoints[j].penetrationDepth - slop, 0.0f);

					float lambda = -c->contactPoints[j].effectiveMassN *
						(c->constraint.JacobianJacobianMult(c->contactPoints[j].jacobianN, c->constraint.velocityJacobian) + b);

					float origNormalImpulseSum = c->contactPoints[j].normalImpulseSum;

					c->contactPoints[j].normalImpulseSum += lambda;
					c->contactPoints[j].normalImpulseSum =
						glm::clamp(c->contactPoints[j].normalImpulseSum, 0.0f, std::numeric_limits<float>::infinity());

					float deltaLambda = c->contactPoints[j].normalImpulseSum - origNormalImpulseSum;

					c->constraint.ApplyImpulse(c->bodyA, c->bodyB, c->contactPoints[j].mMatxjN, deltaLambda);

					if (applyFriction) {
						c->constraint.EvaluateVelocityJacobian(c->bodyA, c->bodyB);

						//float nLambda = c->contactPoints[j]->normalImpulseSum;
						float nLambda = -gravity.y / pointCount;

						//==== solve for tangent 0
						lambda = -c->contactPoints[j].effectiveMassT0 *
							(c->constraint.JacobianJacobianMult(c->contactPoints[j].jacobianT0, c->constraint.velocityJacobian) + 0.0f);

						float origTangent0ImpulseSum = c->contactPoints[j].tangentImpulseSum1;

						c->contactPoints[j].tangentImpulseSum1 += lambda;
						c->contactPoints[j].tangentImpulseSum1 =
							glm::clamp(c->contactPoints[j].tangentImpulseSum1, -mu * nLambda, mu * nLambda);

						deltaLambda = c->contactPoints[j].tangentImpulseSum1 - origTangent0ImpulseSum;

						c->constraint.ApplyImpulse(c->bodyA, c->bodyB, c->contactPoints[j].mMatxjT0, deltaLambda);

						//==== solve for tangent 1
						c->constraint.EvaluateVelocityJacobian(c->bodyA, c->bodyB);

						lambda = -c->contactPoints[j].effectiveMassT1 *
							(c->constraint.JacobianJacobianMult(c->contactPoints[j].jacobianT1, c->constraint.velocityJacobian) + 0.0f);

						float origTangent1ImpulseSum = c->contactPoints[j].tangentImpulseSum2;

						c->contactPoints[j].tangentImpulseSum2 += lambda;
						c->contactPoints[j].tangentImpulseSum2 =
							glm::clamp(c->contactPoints[j].tangentImpulseSum2, -mu * nLambda, mu * nLambda);

						deltaLambda = c->contactPoints[j].tangentImpulseSum2 - origTangent1ImpulseSum;

						c->constraint.ApplyImpulse(c->bodyA, c->bodyB, c->contactPoints[j].mMatxjT1, deltaLambda);
					}
				}
			}
		}

		//===== solve joints
		if (isResolvingJoints)
		{
			for (auto j : joints)
			{
				j->ApplyImpulse();
			}
		}

	}

		{
			//std::cout << "Copy ";
			//Timer t;
			// Copy contacts into previous list
			for (auto c : *colMan->mPrevContacts)
				delete c;

			colMan->mPrevContacts->clear();

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

	ImGui::Begin("Damping Values");
	ImGui::InputFloat("Angular Damp ", &angularDamp);
	ImGui::InputFloat("Linear Damp ", &linearDamp);
	ImGui::End();
}

PhysicsSystem::~PhysicsSystem()
{
}
