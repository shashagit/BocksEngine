/* Start Header -------------------------------------------------------
Copyright (C) 20xx DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the prior
written consent of DigiPen Institute of Technology is prohibited.
File Name: <put file name here>
Purpose: <explain the contents of this file>
Language: <specify language and compiler>
Platform: <specify compiler version, hardware requirements, operating
systems>
Project: <specify student login, class, and assignment. For example:
if foo.boo is in class CS 529 and this file is a part of
assignment 2, then write: CS529_fooboo_2>
Author: <provide your name, student login, and student id>
Creation date: <date on which you created this file>
- End Header --------------------------------------------------------*/

#include "GameObjectManager.h"
#include "GameObject.h"
#include "ObjectFactory.h"

#include "../Components/Transform.h"
#include "../Components/Component.h"
#include "../Components/Mesh.h"
#include "../Components/Body.h"
#include "../Components/Collider.h"
#include "../Components/DebugVector.h"
#include "../Components/Shape.h"

#include <cstdlib>
#include <ctime>

extern ObjectFactory* gpObjectFactory;

GameObjectManager::GameObjectManager() {

	// creating map of components 
	componentMap[TRANSFORM] = new Transform();
	componentMap[BODY] = new Body();
	componentMap[MESH] = new Mesh();
	componentMap[COLLIDER] = new Collider();
	componentMap[DEBUGVEC] = new DebugVector();

	// setting Color array
	Colors[Color::Red] = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f);
	Colors[Color::Green] = glm::vec4(0.0f, 1.0f, 0.0f, 1.0f);
	Colors[Color::Blue] = glm::vec4(0.0f, 0.0f, 1.0f, 1.0f);
	Colors[Color::White] = glm::vec4(1.0f, 1.0f, 1.0f, 1.0f);
}

GameObjectManager::~GameObjectManager() {
	for(auto go: mGameObjects) {
		delete go;
	}
}

void GameObjectManager::DeleteGameObject(GameObject* go) {

	std::vector<GameObject*>::iterator position = std::find(mGameObjects.begin(), mGameObjects.end(), go);
	if (position != mGameObjects.end()) {
		mGameObjects.erase(position);
	}

}

void GameObjectManager::Update()
{
	for (auto go : mGameObjects)
		go->Update();
}

Transform* GameObjectManager::CreateDebugObject()
{
	GameObject* go = new GameObject();
	Transform* pTr = static_cast<Transform*>(go->AddComponent(TRANSFORM));
	pTr->mScale = glm::vec3(0.1f, 0.1f, 0.1f);
	pTr->Update();

	mDebugObjects.push_back(go);

	return pTr;
}
