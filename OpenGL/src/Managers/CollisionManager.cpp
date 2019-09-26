#include "CollisionManager.h"
#include "../Components/Shape.h"


CollisionManager::CollisionManager()
{
}

CollisionManager::~CollisionManager()
{
	Reset();
}

void CollisionManager::Reset()
{
	for (auto contact : mContacts) {
		delete contact;
	}
	
	mContacts.clear();
}
