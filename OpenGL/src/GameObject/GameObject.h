#ifndef OBJECT_H
#define OBJECT_H

#include<vector>

class Component;
class Event;

enum GameObjectID {
	CUBE,
	SPHERE,
	PLANE
};

class GameObject {
	public:
	GameObject();
	~GameObject();

	void Update();
	Component* AddComponent(unsigned int Type);
	Component* GetComponent(unsigned int Type);

	public:
	GameObjectID ID;
	std::vector<Component*> mComponents;

};

#endif