#include "Transform.h"

Transform::Transform() : Component(TRANSFORM), mPos(0)
{
	mRotate = glm::mat4(1.0);
	mScale = glm::vec3(1.0f, 1.0f, 1.0f);
}

Transform* Transform::Create() {
	return new Transform();
}

Transform::~Transform()
{
}

void Transform::Update() {
	model = glm::translate(glm::mat4(1.0f), mPos);
	/*model = glm::rotate(model, glm::radians(mRotate.x), glm::vec3(1.0f, 0.0f, 0.0f));
	model = glm::rotate(model, glm::radians(mRotate.y), glm::vec3(0.0f, 1.0f, 0.0f));
	model = glm::rotate(model, glm::radians(mRotate.z), glm::vec3(0.0f, 0.0f, 1.0f));*/
	model *= mRotate;
	model = glm::scale(model, mScale);
}

void Transform::Serialize(GenericObject<false, Value::ValueType> doc)
{
	if (doc.HasMember("Position")) {
		mPos.x = doc["Position"]["x"].GetFloat();
		mPos.y = doc["Position"]["y"].GetFloat();
		mPos.z = doc["Position"]["z"].GetFloat();
	}

	if (doc.HasMember("Scale")) {
		mScale.x = doc["Scale"]["x"].GetFloat();
		mScale.y = doc["Scale"]["y"].GetFloat();
		mScale.z = doc["Scale"]["z"].GetFloat();
	}

	// update to use mRotate matrix instead of angle values
	//if (doc.HasMember("Angle"))
		//mRotate.x = doc["Angle"]["x"].GetFloat();
}