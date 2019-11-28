
#include "Transform.h"
#include "Shape.h"
#include "Body.h"

#include "../GameObject/GameObjectManager.h"

extern GameObjectManager* gpGoManager;

Body::Body() : Component(BODY)
{
	mPos = glm::vec3(0);
	mPrevPos = glm::vec3(0);
	mTotalForce = glm::vec3(0);
	mVel = glm::vec3(0);

	mRotationMatrix = glm::mat4(1.0f);
	mAngularVel = glm::vec3(0.0f);

	mQuaternion = glm::fquat(0.0f, 0.0f, 0.0f, 1.0f);
}

Body* Body::Create() {
	return new Body();
}

Body::~Body()
{
}

void Body::ApplyForce(glm::vec3 force, glm::vec3 pos) {
	mTotalForce += force;
	mTotalTorque += glm::cross(pos - mPos, force);
}

void Body::Serialize(GenericObject<false, Value::ValueType> d)
{
	if(d.HasMember("Mass"))
		mMass = d["Mass"].GetFloat();

	if (d.HasMember("Velocity")) {
		mVel.x = d["Velocity"]["x"].GetFloat();
		mVel.y = d["Velocity"]["y"].GetFloat();
		mVel.z = d["Velocity"]["z"].GetFloat();
	}
	else {
		mVel = glm::vec3(0);
	}

	//debugVector = gpGoManager->CreateDebugObject(White);
}

void Body::Initialize()
{
	Transform *pTr = static_cast<Transform*>(mpOwner->GetComponent(TRANSFORM));
	if (pTr != nullptr) {
		mPrevPos = mPos;
		mPos = pTr->mPos;
	}

	mQuaternion = glm::toQuat(pTr->mRotate);
	//mAngularVel.x = 0.1f;
	//mAngularVel.y = 0.1f;
	glm::mat3 inertia = glm::mat3(0.0f);
	if (mMass != std::numeric_limits<float>::infinity()) {
		// setup body inertia tensor (only for AABB)
		inertia[0][0] = mMass / 12.0f * (pTr->mScale.y * pTr->mScale.y + pTr->mScale.z * pTr->mScale.z);
		inertia[1][1] = mMass / 12.0f * (pTr->mScale.x * pTr->mScale.x + pTr->mScale.z * pTr->mScale.z);
		inertia[2][2] = mMass / 12.0f * (pTr->mScale.y * pTr->mScale.y + pTr->mScale.x * pTr->mScale.x);
		mInertiaBodyInverse = glm::inverse(inertia);

		mInvMass = 1.0f / mMass;
	} else
	{
		mMass = std::numeric_limits<float>::infinity();
		mInvMass = 0.0f;
		mInertiaBodyInverse = glm::mat3(0.0f);
		return;
	}


}

