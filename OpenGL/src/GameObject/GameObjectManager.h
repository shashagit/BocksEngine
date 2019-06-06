#pragma once
#include<vector>
#include<unordered_map>
#include"..\Components\Component.h"
#include "GameObject.h"

enum Color {
	Red,
	Blue,
	Green,
	White
};

class Transform;

class GameObjectManager {
	public:
	GameObjectManager();
	~GameObjectManager();
	
	public:
		void DeleteGameObject(GameObject* go);
		void Update();

		Transform* CreateDebugObject();

		glm::vec4 Colors[4];

	std::vector<GameObject*> mGameObjects;
	std::vector<GameObject*> mDebugObjects;
	std::unordered_map<COMPONENT_TYPE, Component*> componentMap;
};