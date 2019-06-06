#pragma once

#include <algorithm>
#include <list>
#include <vector>

#include "../Components/Collider.h"
#include "../Components/Shape.h"

typedef std::pair<Collider*, Collider*> ColliderPair;

class Broadphase
{
public:
	
	virtual void AddCollider(Collider *) = 0;

	virtual void Update() = 0;

	//virtual const std::list<ColliderPair>& CalculatePairs() = 0;
	virtual void CalculatePairs() = 0;

	virtual void CollisionQuery(const Shape& shape, std::vector<Collider*>& colliders) = 0;
};

