
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

	//mQuaternion = glm::fquat(0.0f, 0.0f, 0.0f, 1.0f);
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

	if (mMass != 0.0f) {
		mInvMass = 1.0f / mMass;
	}
	else {
		mInvMass = 0.0f;
	}

	if (d.HasMember("Velocity")) {
		mVel.x = d["Velocity"]["x"].GetFloat();
		mVel.y = d["Velocity"]["y"].GetFloat();
		mVel.z = d["Velocity"]["z"].GetFloat();
	}
	else {
		mVel = glm::vec3(0);
	}

	if (d.HasMember("Quat")) {
		mQuaternion.x = d["Quat"]["x"].GetFloat();
		mQuaternion.y = d["Quat"]["y"].GetFloat();
		mQuaternion.z = d["Quat"]["z"].GetFloat();
		mQuaternion.w = d["Quat"]["w"].GetFloat();
	}

	mQuaternion = glm::normalize(mQuaternion);
	//debugVector = gpGoManager->CreateDebugObject(White);
}

void Body::Initialize()
{
	Transform *pTr = static_cast<Transform*>(mpOwner->GetComponent(TRANSFORM));
	if (pTr != nullptr) {
		mPrevPos = mPos;
		mPos = pTr->mPos;
	}

	//mAngularVel.x = 0.1f;
	//mAngularVel.y = 0.1f;

	// setup body inertia tensor (only for AABB)
	glm::mat3 inertia = glm::mat3(0.0f);
	inertia[0][0] = mMass / 12.0f * (pTr->mScale.y * pTr->mScale.y + pTr->mScale.z * pTr->mScale.z);
	inertia[1][1] = mMass / 12.0f * (pTr->mScale.x * pTr->mScale.x + pTr->mScale.z * pTr->mScale.z);
	inertia[2][2] = mMass / 12.0f * (pTr->mScale.y * pTr->mScale.y + pTr->mScale.x * pTr->mScale.x);

	mInertiaBodyInverse = glm::inverse(inertia);
}

