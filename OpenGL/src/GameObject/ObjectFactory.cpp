#include "ObjectFactory.h"
#include "../Components/Transform.h"
#include "../Components/Component.h"
#include "../Components/Body.h"
#include "../Components/Collider.h"

#include "GameObjectManager.h"
#include "GameObject.h"

#include <fstream>
#include <sstream>
#include <iostream>
#include "rapidjson/stringbuffer.h"
#include "rapidjson/document.h"
#include "rapidjson/writer.h"

extern GameObjectManager* gpGoManager;


ObjectFactory::ObjectFactory()
{
}


ObjectFactory::~ObjectFactory()
{
}

void ObjectFactory::LoadBigLevel() {

	int dim = 5;
	for (int i = 0; i < dim; ++i) {
		for (int j = 0; j < dim; ++j) {
			for (int k = 0; k < dim; ++k) {
				GameObject* go = LoadObject("Cube");
				Body* pB = static_cast<Body*>(go->GetComponent(BODY));
				Transform* pTr = static_cast<Transform*>(go->GetComponent(TRANSFORM));
				pTr->mPos = glm::vec3(2.0f * i, 2.0f*j, 2.0f * k);
				pB->Initialize();

				//physics->dAABBTree.AddCollider(static_cast<Collider*>(go->GetComponent(COLLIDER)));
			}
		}
	}
	dim = 10;
	for (int j = 0; j < dim; ++j) {
		for (int k = 0; k < dim; ++k) {
			GameObject* go = LoadObject("Plane");
			Body* pB = static_cast<Body*>(go->GetComponent(BODY));
			Transform* pTr = static_cast<Transform*>(go->GetComponent(TRANSFORM));
			pTr->mPos = glm::vec3(1.0f * j, -10.0f, 1.0f * k);
			pB->Initialize();

			//physics->dAABBTree.AddCollider(static_cast<Collider*>(go->GetComponent(COLLIDER)));
		}
	}
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
