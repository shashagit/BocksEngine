#include "DynamicAABBTree.h"



DynamicAABBTree::DynamicAABBTree()
{
	root = nullptr;
	fatteningMargin = 0.1f; // 10% fattened
}


DynamicAABBTree::~DynamicAABBTree()
{
}

void DynamicAABBTree::RemoveNode(Node* node) {
	Node* parent = node->parent;
	if (parent) {
		// if parent exists we need to switch sibling to parent position
		Node* sibling = node->GetSibling();
		if (parent->parent) {
			sibling->parent = parent->parent;
			(parent == parent->parent->left ? parent->parent->left : parent->parent->right) = sibling;
		}
		else {
			root = sibling;
			sibling->parent = nullptr;
		}
	}
	else {
		root = nullptr;
		delete node;
	}
}

void DynamicAABBTree::DeleteTree() {
	root = nullptr;
	nodecount = 0;
	colliderPairs.clear();
}

void DynamicAABBTree::RemoveCollider() {
	// this needs a map between the collider and the generated nodes
	// TODO when updating to Vector based tree create a map for this
}

// Insert method for the tree
void DynamicAABBTree::AddCollider(Collider *col) {
	if (root) {
		Node* node = new Node();
		node->mClientData = col;
		//node->aabb = static_cast<ShapeAABB*>(col->mpShape);
		node->UpdateAABB(fatteningMargin);
		InsertNode(node, &root);
	}
	// no root in the tree
	else {
		root = new Node();
		root->mClientData = col;
		//root->aabb = static_cast<ShapeAABB*>(col->mpShape);
		root->UpdateAABB(fatteningMargin);
	}
}

void DynamicAABBTree::InsertNode(Node* node, Node** parent) {
	Node* p = *parent;
	// parent is leaf, set it to left and new node to right
	if (p->IsLeaf()) {
		Node* newParent = new Node();
		newParent->parent = p->parent;
		p->parent = newParent;
		node->parent = newParent;
		newParent->left = p;
		newParent->right = node;

		*parent = newParent;
	}
	// parent is internal node, select node to insert
	else {
		Node** selectedNode = SelectNode(node, &p->left, &p->right);
		InsertNode(node, selectedNode);
	}

	SyncHierarchy(*parent);
}

// Helper function to the insert method
Node** DynamicAABBTree::SelectNode(Node* insertingNode, Node** left, Node** right) {
	ShapeAABB mergedLeft(glm::vec3(0), glm::vec3(0)), mergedRight(glm::vec3(0), glm::vec3(0));

	mergedLeft.Merge(*insertingNode->aabb, *(*left)->aabb);
	mergedRight.Merge(*insertingNode->aabb, *(*right)->aabb);

	if (mergedLeft.GetSurfaceArea() > mergedRight.GetSurfaceArea()) {
		return right;
	}
		return left;
}

// Update Height and AABBs of the entire tree traversing upwards
void DynamicAABBTree::SyncHierarchy(Node* parent){
	Node* p = parent;
	while (p != nullptr) {
		Node* l = parent->left;
		Node* r = parent->right;

		p->height = 1 + std::max(l->height, r->height);
		p->aabb->Merge(*l->aabb, *r->aabb);

		p = p->parent;
	}
}

// recursive helper to find all the nodes that need updating
void FindUnfitNodes(Node* node, std::vector<Node*>& nodeList) {
	if (node->IsLeaf()) {
		Collider* col = static_cast<Collider*>(node->mClientData);
		if (!node->aabb->Contains(*static_cast<ShapeAABB*>(col->mpShape))) {
			nodeList.push_back(node);
		}
		
	}
	else {
		FindUnfitNodes(node->left, nodeList);
		FindUnfitNodes(node->right, nodeList);
	}
}

void DynamicAABBTree::Update() {
	if (root) {
		if (root->IsLeaf()) {
			root->UpdateAABB(fatteningMargin);
		}
		else {
			// find all leaf nodes that no longer fit their AABBs
			unfitNodes.clear();
			FindUnfitNodes(root, unfitNodes);

			// re-insert all the unfit nodes to make them fit
			for (Node* node : unfitNodes) {
				Node* p = node->parent;
				Node* s = node->GetSibling();
				Node** gp;
				// if grandparent exists, find if parent was left or right
				if (p->parent) {
					gp = (p == p->parent->left) ? &p->parent->left : &p->parent->right;
				}
				else {
					gp = &root;
				}

				// replace parent with sibling
				s->parent = p->parent ? p->parent : nullptr;

				*gp = s;

				delete p;

				SyncHierarchy(s->parent);

				// reinsert
				node->UpdateAABB(fatteningMargin);
				InsertNode(node, &root);
			}
		}
	}
}


void DynamicAABBTree::CalculatePairs() {
	colliderPairs.clear();

	if (!root || root->IsLeaf()) {
		return;
	}

	SelfQuery(root);
}

void DynamicAABBTree::SelfQuery(Node* A, Node* B) {
	if (A->IsLeaf()) {
		if (B->IsLeaf()) {
			//ShapeAABB* A_Shape = static_cast<ShapeAABB*>(static_cast<Collider*>(A->mClientData)->mpShape);
			//ShapeAABB* B_Shape = static_cast<ShapeAABB*>(static_cast<Collider*>(B->mClientData)->mpShape);
			// both are leaf
			//if (A_Shape->Collides(B_Shape)) {
			if(A->aabb->Collides(B->aabb)) {
				// push the colliding pairs
				colliderPairs.push_front(std::make_pair(static_cast<Collider*>(A->mClientData),
					static_cast<Collider*>(B->mClientData)));
			}
		}
		else {
			// one internal, one leaf
			SelfQuery(B->left, A);
			SelfQuery(B->right, A);

			//SelfQuery(B);
		}
	}
	else {
		if (B->IsLeaf()) {
			// one internal, one leaf
			SelfQuery(A->left, B);
			SelfQuery(A->right, B);

			//SelfQuery(A);
		}
		else {
			// both internal
			SplitNodes(A, B);
		}
	}
}

void DynamicAABBTree::SplitNodes(Node* A, Node* B) {
	// check for all 4 combinations of children
	SelfQuery(A->left, B->left);
	SelfQuery(A->right, B->left);
	SelfQuery(A->left, B->right);
	SelfQuery(A->right, B->right);
}

void DynamicAABBTree::SelfQuery(Node* node) {
	if (node->IsLeaf()) {
		return;
	}
	else {
		// check the children against each other
		SelfQuery(node->left, node->right);

		// recurse down both subtrees
		SelfQuery(node->left);
		SelfQuery(node->right);
	}
}

void DynamicAABBTree::CollisionQuery(const Shape& shape, std::vector<Collider*>& colliders) {

}