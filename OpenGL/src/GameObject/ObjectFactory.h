#pragma once
#include <unordered_map>


class GameObject;

class ObjectFactory
{
public:
	ObjectFactory();
	~ObjectFactory();

	void LoadLevel(const char* pFileName);
	GameObject* CreateObject(const char * str, const char * pFileName);
	GameObject* LoadObject(const char* pFileName);

	void LoadBigLevel();

};

