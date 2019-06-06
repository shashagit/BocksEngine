#include"GameObject.h"
#include "../Components/Transform.h"

#include "GameObjectManager.h"

extern GameObjectManager* gpGoManager;

GameObject::GameObject() {

}

GameObject::~GameObject() {
	for(auto c: mComponents)
		delete c;
	mComponents.clear();
}

void GameObject::Update() {
	for (auto c : mComponents)
		c->Update();
}

Component* GameObject::AddComponent(unsigned int Type) {
	Component* pNewComponent = nullptr;
	

	pNewComponent = gpGoManager->componentMap[static_cast<COMPONENT_TYPE>(Type)]->Create();
	
	if(pNewComponent != nullptr) {
		mComponents.push_back(pNewComponent);
		pNewComponent->mpOwner = this;
	}

	return pNewComponent;
}

Component* GameObject::GetComponent(unsigned int Type) {
	for(auto c: mComponents) {
		if(c->GetType() == Type)
			return c;
	}
	return nullptr;
}