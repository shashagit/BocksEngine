#pragma once
#include "Broadphase.h"

class NSquaredBroadphase : public Broadphase
{
public:
	void AddCollider(Collider *col);

	void Update();

	void CalculatePairs();
	
	std::list<ColliderPair>& GetPairs() {
		return colliderPairs;
	}

	void RemoveColliders();

	void CollisionQuery(const Shape& shape, std::vector<Collider*>& colliders);

private:
	std::vector<Collider*> AABBlist;
	std::list<ColliderPair> colliderPairs;
};

