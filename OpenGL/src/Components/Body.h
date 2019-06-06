#pragma once

#include "rapidjson/document.h"
#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/quaternion.hpp"

#include "Component.h"

class Sprite;
class Transform;

class Body : public Component
{
public:
	Body();
	~Body();

	void Serialize(GenericObject<false, Value::ValueType> doc);
	Body* Create();
	void Initialize();
	void ApplyForce(glm::vec3 force, glm::vec3 pos);

public:
	glm::vec3 mPos;
	glm::vec3 mPrevPos;

	// better to use Linear and Angular Momentum in place of this
	glm::vec3 mVel;
	glm::vec3 mAngularVel;

	glm::vec3 mTotalForce;
	glm::vec3 mTotalTorque;

	glm::mat3 mInertiaBodyInverse;
	glm::mat3 mInertiaWorldInverse;

	glm::fquat mQuaternion; // can be converted to Euler Angles

	glm::mat3 mRotationMatrix; // for calculating world Inertia tensor

	float mMass, mInvMass;

};

