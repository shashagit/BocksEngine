#include "CollisionManager.h"
#include "../Components/Shape.h"


CollisionManager::CollisionManager()
{
	mContacts = new std::list<ContactManifold*>();
	mPrevContacts = new std::list<ContactManifold*>();
}

CollisionManager::~CollisionManager()
{
	Reset();
}

void CollisionManager::Reset()
{
	/*for (auto contact : mContacts) {
		delete contact;
	}*/
	
	mContacts = new std::list<ContactManifold*>();
}
