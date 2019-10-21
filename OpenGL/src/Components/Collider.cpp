#include "Collider.h"
#include "Transform.h"
#include "Body.h"
#include "Shape.h"

Collider::Collider() : Component(COLLIDER), mpShape(nullptr), isTrigger(false)
{
	mpShape = new ShapeAABB(glm::vec3(0), glm::vec3(0));
	mpShape->mpOwnerCollider = this;

	mpLocalShape = new ShapeAABB(glm::vec3(0), glm::vec3(0));
	
	coeffRestitution = 0.2f; // bounce
	coeffStaticFriction = 0.9f; // to start sliding
	coeffDynamicFriction = 0.75f; // while sliding

	mDebugMatrix = glm::mat4(1.0f);
	colliderColor = Green;


	//mDebugMatrix = reinterpret_cast<glm::mat4 *> (_aligned_malloc(sizeof(glm::mat4), 16));

}

Collider* Collider::Create() {
	return new Collider();
}

void Collider::Serialize(GenericObject<false, Value::ValueType> d) {

	mpTr = static_cast<Transform*>(mpOwner->GetComponent(TRANSFORM));
	mpBody = static_cast<Body*>(mpOwner->GetComponent(BODY));
	
	SetMeshData();
	
	//meshData.faces[0];

	// update local shape
	static_cast<ShapeAABB*>(mpLocalShape)->mMin = glm::vec3(-0.5f, -0.5f, -0.5f) * (mpTr->mScale);// +mpTr->mPos;
	static_cast<ShapeAABB*>(mpLocalShape)->mMax = glm::vec3(0.5f, 0.5f, 0.5f) * (mpTr->mScale);// +mpTr->mPos;
}

void Collider::Update() {
	UpdateShape();
	if (isColliding) {
		colliderColor = Red;
	}
	else {
		colliderColor = Green;
	}
}

void Collider::SetMeshData()
{
	// generate vertices
	meshData.AddVertex(mpTr->mScale * glm::vec3(-0.5f, -0.5f, 0.5f));
	meshData.AddVertex(mpTr->mScale * glm::vec3(0.5f, -0.5f, 0.5f));
	meshData.AddVertex(mpTr->mScale * glm::vec3(0.5f, 0.5f, 0.5f));
	meshData.AddVertex(mpTr->mScale * glm::vec3(-0.5f, 0.5f, 0.5f));
	meshData.AddVertex(mpTr->mScale * glm::vec3(-0.5f, -0.5f, -0.5f));
	meshData.AddVertex(mpTr->mScale * glm::vec3(0.5f, -0.5f, -0.5f));
	meshData.AddVertex(mpTr->mScale * glm::vec3(0.5f, 0.5f, -0.5f));
	meshData.AddVertex(mpTr->mScale * glm::vec3(-0.5f, 0.5f, -0.5f));

	// generate (quadrilateral) faces
	std::vector<int> faceVerts;
	faceVerts.clear();
	faceVerts.push_back(0);
	faceVerts.push_back(1);
	faceVerts.push_back(2);
	faceVerts.push_back(3);
	meshData.AddFace(faceVerts);

	faceVerts.clear();
	faceVerts.push_back(7);
	faceVerts.push_back(6);
	faceVerts.push_back(5);
	faceVerts.push_back(4);
	meshData.AddFace(faceVerts);
	faceVerts.clear();
	faceVerts.push_back(1);
	faceVerts.push_back(0);
	faceVerts.push_back(4);
	faceVerts.push_back(5);
	meshData.AddFace(faceVerts);
	faceVerts.clear();
	faceVerts.push_back(2);
	faceVerts.push_back(1);
	faceVerts.push_back(5);
	faceVerts.push_back(6);
	meshData.AddFace(faceVerts);
	faceVerts.clear();
	faceVerts.push_back(3);
	faceVerts.push_back(2);
	faceVerts.push_back(6);
	faceVerts.push_back(7);
	meshData.AddFace(faceVerts);
	faceVerts.clear();
	faceVerts.push_back(0);
	faceVerts.push_back(3);
	faceVerts.push_back(7);
	faceVerts.push_back(4);
	meshData.AddFace(faceVerts);
}

void Collider::UpdateShape() {
	glm::vec3 extents = static_cast<ShapeAABB*>(mpLocalShape)->GetHalfExtents();
	glm::vec3 x = glm::vec3(extents.x, 0.0f, 0.0f);
	glm::vec3 y = glm::vec3(0.0f, extents.y, 0.0f);
	glm::vec3 z = glm::vec3(0.0f, 0.0f, extents.z);
	glm::vec3 rotatedExtents = abs(glm::mat3(mpTr->mRotate) * x)+
		abs(glm::mat3(mpTr->mRotate) * y)+
		abs(glm::mat3(mpTr->mRotate) * z);

	// based on normalized body vertices
	static_cast<ShapeAABB*>(mpShape)->mMin = glm::vec3(-rotatedExtents.x, -rotatedExtents.y, -rotatedExtents.z) + mpBody->mPos;
	static_cast<ShapeAABB*>(mpShape)->mMax = glm::vec3(rotatedExtents.x, rotatedExtents.y, rotatedExtents.z) + mpBody->mPos;
}

Collider::~Collider()
{
	//_aligned_free(mDebugMatrix);
}
