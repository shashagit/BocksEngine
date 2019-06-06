#pragma once

#include "Component.h"

class Transform : public Component
{
public:
	Transform();
	~Transform();
	
	Transform* Create();

	void Update();
	void Serialize(GenericObject<false, Value::ValueType>);
public:
	glm::vec3 mPos;
	glm::vec3 mScale;
	glm::mat4 mRotate;

	glm::mat4 model;
};