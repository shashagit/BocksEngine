#include "ObjectFactory.h"
#include "../Components/Transform.h"
#include "../Components/Component.h"
#include "../Components/Body.h"
#include "../Components/Collider.h"
#include "../Articulation/Joint.h"

#include "GameObjectManager.h"
#include "GameObject.h"
#include "../Managers/PhysicsSystem.h"

#include <fstream>
#include <sstream>
#include <iostream>
#include "rapidjson/stringbuffer.h"
#include "rapidjson/document.h"
#include "rapidjson/writer.h"

extern GameObjectManager* gpGoManager;
extern PhysicsSystem* physics;

extern float angularDamp;
extern float linearDamp;

ObjectFactory::ObjectFactory()
{
}


ObjectFactory::~ObjectFactory()
{
}

void ObjectFactory::LoadBigLevel() {

	int dim =5;
	int height = 5;
	for (int i = 0; i < dim; ++i) {
		for (int j = 0; j < dim; ++j) {
			for (int k = 0; k < height; ++k) {
				GameObject* go = LoadObject("Cube");
				Body* pB = static_cast<Body*>(go->GetComponent(BODY));
				Transform* pTr = static_cast<Transform*>(go->GetComponent(TRANSFORM));
				pTr->mPos = glm::vec3(2.0f * i, 1.2f*k, 2.0f * j);
				pB->Initialize();

				//physics->dAABBTree.AddCollider(static_cast<Collider*>(go->GetComponent(COLLIDER)));
			}
		}
	}
	GameObject* go = LoadObject("Plane");
	Body* pB = static_cast<Body*>(go->GetComponent(BODY));
	Transform* pTr = static_cast<Transform*>(go->GetComponent(TRANSFORM));
	pTr->mPos = glm::vec3(4.0f, -5.0f, 4.0f);
	pB->Initialize();
	//for (int i = 0; i < dim; ++i) {
	//	for (int j = 0; j < dim; ++j) {
	//			GameObject* go = LoadObject("Plane");
	//			Body* pB = static_cast<Body*>(go->GetComponent(BODY));
	//			Transform* pTr = static_cast<Transform*>(go->GetComponent(TRANSFORM));
	//			pTr->mPos = glm::vec3(2.0f * i, -5.0f, 2.0f * j);
	//			pB->Initialize();
	//			//physics->dAABBTree.AddCollider(static_cast<Collider*>(go->GetComponent(COLLIDER)));
	//	}
	//}
	//

	// set angular damping
	angularDamp = 0.05f;
}

void ObjectFactory::LoadLevel(const char * pFileName)
{
	std::string fullPath = "res\\Levels\\";
	fullPath += pFileName;
	fullPath += ".json";

	std::ifstream file(fullPath);
	std::stringstream ss;
	ss << file.rdbuf();
	std::string temp = ss.str();
	const char* str = temp.c_str();

	Document document;
	document.Parse(str);
	StringBuffer buffer;
	Writer<StringBuffer> writer(buffer);
	const rapidjson::Value& V = document;
	for (Value::ConstMemberIterator iter = V.MemberBegin(); iter != V.MemberEnd(); ++iter) {
		if (iter->value.IsObject()) {// means it refers to an object
			GameObject* pGO = LoadObject(iter->name.GetString());
			// set position according to data in level file
			Transform* pTr = static_cast<Transform*>(pGO->GetComponent(TRANSFORM));
			
			if (pTr) {
				writer.Reset(buffer);
				iter->value.Accept(writer);
				Document innerDoc;
				innerDoc.Parse(buffer.GetString());
				pTr->Serialize(innerDoc["Transform"].GetObject());
			}
			buffer.Clear();

			Body* pBr = static_cast<Body*>(pGO->GetComponent(BODY));
			if (pBr != nullptr) {
				pBr->Initialize();
				Collider* pCol = static_cast<Collider*>(pGO->GetComponent(COLLIDER));
				pCol->UpdateShape();
			}
		}
	}
}

GameObject* ObjectFactory::LoadObject(const char * pFileName)
{
	std::string fullPath = "res\\Objects\\";
	fullPath = fullPath + pFileName + ".json";

	std::ifstream file(fullPath);
	std::stringstream ss;
	ss << file.rdbuf();
	std::string temp = ss.str();
	const char* str = temp.c_str();

	return CreateObject(str, pFileName);
}

void ObjectFactory::LoadBallJointLevel()
{
	GameObject* pGO1 = LoadObject("Cube");
	GameObject* pGO2 = LoadObject("Cube");

	Transform* pTr2 = static_cast<Transform*>(pGO2->GetComponent(TRANSFORM));
	pTr2->mPos = glm::vec3(1.1f, 0.0f, 0.0f);
	Body* pB1 = static_cast<Body*>(pGO1->GetComponent(BODY));
	Body* pB2 = static_cast<Body*>(pGO2->GetComponent(BODY));

	pB1->Initialize();
	pB2->Initialize();

	BallJoint* j = new BallJoint(pB1, pB2, glm::vec3(0.5f, 0.5f, 0.5f ));
	for (auto j : physics->joints)
	{
		delete j;
	}
	physics->joints.clear();
	physics->joints.push_back(j);

	physics->isResolvingContacts = true;
	physics->isResolvingJoints = true;

	GameObject* go = LoadObject("Plane");
	Body* pB = static_cast<Body*>(go->GetComponent(BODY));
	Transform* pTr = static_cast<Transform*>(go->GetComponent(TRANSFORM));
	pTr->mPos = glm::vec3(0.0f, -5.0f, 0.0f);
	pB->Initialize();

	// set angular damping
	angularDamp = 10.0f;
}

void ObjectFactory::LoadJointLevel()
{
	GameObject* pGO1 = LoadObject("Cube");
	GameObject* pGO2 = LoadObject("Cube");

	Transform* pTr2 = static_cast<Transform*>(pGO2->GetComponent(TRANSFORM));
	pTr2->mPos = glm::vec3(1.1f, 0.0f, 0.0f);
	Body* pB1 = static_cast<Body*>(pGO1->GetComponent(BODY));
	Body* pB2 = static_cast<Body*>(pGO2->GetComponent(BODY));

	pB1->Initialize();
	pB2->Initialize();

	HingeJoint* j = new HingeJoint(pB1, pB2, glm::vec3(0.55f, 0.55f, 0.55f), glm::vec3(0.55f, 0.55f, -0.55f));
	for (auto j : physics->joints)
	{
		delete j;
	}
	physics->joints.clear();
	physics->joints.push_back(j);

	physics->isResolvingContacts = true;
	physics->isResolvingJoints = true;

	GameObject* go = LoadObject("Plane");
	Body* pB = static_cast<Body*>(go->GetComponent(BODY));
	Transform* pTr = static_cast<Transform*>(go->GetComponent(TRANSFORM));
	pTr->mPos = glm::vec3(0.0f, -5.0f, 0.0f);
	pB->Initialize();

	// set angular damping
	angularDamp = 10.0f;
}

void ObjectFactory::LoadRopeJointLevel()
{
	GameObject* pGO1 = LoadObject("Cube");
	GameObject* pGO2 = LoadObject("Cube");

	Transform* pTr2 = static_cast<Transform*>(pGO2->GetComponent(TRANSFORM));
	pTr2->mPos = glm::vec3(1.1f, 0.0f, 0.0f);
	Body* pB1 = static_cast<Body*>(pGO1->GetComponent(BODY));
	Body* pB2 = static_cast<Body*>(pGO2->GetComponent(BODY));

	pB1->Initialize();
	pB2->Initialize();

	RopeJoint* j = new RopeJoint(pB1, pB2, 3.0f);
	for (auto j : physics->joints)
	{
		delete j;
	}
	physics->joints.clear();
	physics->joints.push_back(j);

	physics->isResolvingContacts = true;
	physics->isResolvingJoints = true;

	GameObject* go = LoadObject("Plane");
	Body* pB = static_cast<Body*>(go->GetComponent(BODY));
	Transform* pTr = static_cast<Transform*>(go->GetComponent(TRANSFORM));
	pTr->mPos = glm::vec3(0.0f, -5.0f, 0.0f);
	pB->Initialize();

	// set angular damping
	angularDamp = 0.05f;
}

void ObjectFactory::LoadBridgeLevel()
{
	GameObject* pGO1 = LoadObject("Plank");
	Transform* pTr1 = static_cast<Transform*>(pGO1->GetComponent(TRANSFORM));
	pTr1->mPos = glm::vec3(0.0f, 5.0f, 0.0f);
	Body* pB1 = static_cast<Body*>(pGO1->GetComponent(BODY));
	pB1->mMass = std::numeric_limits<float>::infinity();
	pB1->Initialize();
	for (auto j : physics->joints)
	{
		delete j;
	}
	physics->joints.clear();
	for (int i = 1; i < 15; ++i)
	{
		GameObject* pGO2 = LoadObject("Plank");

		Transform* pTr2 = static_cast<Transform*>(pGO2->GetComponent(TRANSFORM));
		pTr2->mPos = glm::vec3(1.1f * i, 5.0f, 0.0f);
		Body* pB2 = static_cast<Body*>(pGO2->GetComponent(BODY));

		pB2->Initialize();

		HingeJoint* j = new HingeJoint(pB1, pB2, glm::vec3((2*i-1)*0.55f, 5.55f, 0.55f), glm::vec3((2*i-1)*0.55f, 5.55f, -0.55f));
		
		physics->joints.push_back(j);
		
		pB1 = pB2;
	}
	physics->isResolvingContacts = true;
	physics->isResolvingJoints = true;

	pB1->mMass = std::numeric_limits<float>::infinity();
	pB1->Initialize();
	
	GameObject* go = LoadObject("Plane");
	Body* pB = static_cast<Body*>(go->GetComponent(BODY));
	Transform* pTr = static_cast<Transform*>(go->GetComponent(TRANSFORM));
	pTr->mPos = glm::vec3(0.0f, -5.0f, 0.0f);
	pB->Initialize();

	// set angular damping
	angularDamp = 100.0f;
}

GameObject* ObjectFactory::CreateObject(const char * str, const char * pFileName) {
	GameObject* pNewGameObject = nullptr;
	Document document;
	document.Parse(str);
	
	//std::cout << str << std::endl;

	assert(document.IsObject());
	pNewGameObject = new GameObject();
	
	const rapidjson::Value& V = document;
	for (Value::ConstMemberIterator iter = V.MemberBegin(); iter != V.MemberEnd(); ++iter) {
		
		std::string componentName = iter->name.GetString();


		Component *pNewComponent = nullptr;
		if ("Transform" == componentName) {
			pNewComponent = pNewGameObject->AddComponent(TRANSFORM);
			pNewComponent->Serialize(document["Transform"].GetObject());
		}
		else if ("Body" == componentName) {
			pNewComponent = pNewGameObject->AddComponent(BODY);
			pNewComponent->Serialize(document["Body"].GetObject());
		}
		else if ("Mesh" == componentName) {
			pNewComponent = pNewGameObject->AddComponent(MESH);
			pNewComponent->Serialize(document["Mesh"].GetObject());
		}
		else if ("Collider" == componentName) {
			pNewComponent = pNewGameObject->AddComponent(COLLIDER);
			pNewComponent->Serialize(document["Collider"].GetObject());
		}
	}
		
	gpGoManager->mGameObjects.push_back(pNewGameObject);
		
	return pNewGameObject;
}
