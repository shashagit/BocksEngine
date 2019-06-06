#pragma once

#include "Broadphase.h"

class Node {
public:
	Node() {
		aabb = new ShapeAABB(glm::vec3(0), glm::vec3(0));
		left = nullptr;
		right = nullptr;
		parent = nullptr;

		height = 0;
		mClientData = nullptr;
	}

	ShapeAABB* aabb;
	Node* left;
	Node* right;
	Node* parent;

	int height;
	void* mClientData;
	
	Node* GetSibling() {
		return this == parent->left ? parent->right : parent->left;
	}

	bool IsLeaf(void) const
	{
		return left == nullptr;
	}

	void UpdateAABB(float fat) {
		if (IsLeaf()) {
			glm::vec3 fatMargin = glm::vec3(fat, fat, fat);
			aabb->mMin = static_cast<ShapeAABB*>(static_cast<Collider*>(mClientData)->mpShape)->mMin
				- fatMargin;
			aabb->mMax = static_cast<ShapeAABB*>(static_cast<Collider*>(mClientData)->mpShape)->mMax
				+ fatMargin;
		}
		else {
			aabb->Merge(*left->aabb, *right->aabb);
		}
	}
};

class DynamicAABBTree : public Broadphase
{
public:
	DynamicAABBTree();
	~DynamicAABBTree();

	void RemoveCollider();

	void AddCollider(Collider *col);

	void Update();

	//virtual const std::list<ColliderPair>& CalculatePairs() = 0;
	void CalculatePairs();

	void DeleteTree();

	void SyncHierarchy(Node* parent);

	void CollisionQuery(const Shape& shape, std::vector<Collider*>& colliders);

	Node* GetRoot() {
		return root;
	}

	std::list<ColliderPair>& GetPairs() {
		return colliderPairs;
	}

private:
	void InsertNode(Node* node, Node** parent);
	Node** SelectNode(Node* insertingNode, Node** left, Node** right);
	void RemoveNode(Node* node);
	std::vector<Node*> unfitNodes;

	// ComputePair 
	std::list<ColliderPair> colliderPairs;
	void SelfQuery(Node* A, Node* B);
	void SplitNodes(Node*, Node*);
	void SelfQuery(Node*);

	Node* root;
	int nodecount;
	float fatteningMargin;
};

