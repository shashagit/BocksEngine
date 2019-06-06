#include "NSquaredBroadphase.h"
#include "../GameObject/GameObjectManager.h"
#include "../Managers/CollisionManager.h"

extern GameObjectManager* gpGoManager;
extern CollisionManager* colMan;

void NSquaredBroadphase::AddCollider(Collider *col) {
	AABBlist.push_back(col);
}

void NSquaredBroadphase::Update() {
	// don't need to update anything
}

void NSquaredBroadphase::RemoveColliders() {
	AABBlist.clear();
	colliderPairs.clear();
}

void NSquaredBroadphase::CalculatePairs() {

	colliderPairs.clear();

	auto pObj1 = AABBlist.begin();
	auto pObjLast = AABBlist.end();

	for (; pObj1 != pObjLast; ++pObj1) {
		Collider* pCol1 = static_cast<Collider*>(*pObj1);
		for (auto pObj2 = pObj1 + 1; pObj2 != pObjLast; ++pObj2) {
			Collider* pCol2 = static_cast<Collider*>(*pObj2);

			// if bodies are same, continue
			if (pCol1->mpBody == pCol2->mpBody) {
				continue;
			}

			// if sum of inverse masses = 0, then don't check for collision
			if (pCol1->mpBody->mInvMass + pCol2->mpBody->mInvMass == 0)
			{
				continue;
			}

			// Instead of this it is possible to only create pairs of 
			// colliding colliders here and then generate contact at a later part

			// creates contacts with pairs if collides
			ShapeAABB* A_Shape = static_cast<ShapeAABB*>(pCol1->mpShape);
			ShapeAABB* B_Shape = static_cast<ShapeAABB*>(pCol2->mpShape);

			if (A_Shape->Collides(B_Shape)) {
				colliderPairs.push_front(ColliderPair(pCol1, pCol2));
			}
		}
	}
}

void NSquaredBroadphase::CollisionQuery(const Shape& shape, std::vector<Collider*>& colliders) {


}