#pragma once

#include "rapidjson/document.h"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>

#include "../GameObject/GameObject.h"

using namespace rapidjson;

enum COMPONENT_TYPE
{
	TRANSFORM,
	BODY,
	MESH,
	COLLIDER,
	DEBUGVEC
};

class Component
{
public:
	Component(unsigned int type);
	~Component();
	virtual void Update() {}
	virtual Component* Create() = 0;
	virtual void Serialize(GenericObject<false, Value::ValueType>) = 0;
	unsigned int GetType() {
		return (mType);
	}
public:
	GameObject *mpOwner;
	unsigned int mType;
};