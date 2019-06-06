#pragma once
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>

class Collider;
enum ShapeType
{
	CIRCLE,
	AABB,
	SHAPESNUM
};

class Shape
{
public:

	Shape(ShapeType type)
	{
		mType = type;
		mpOwnerCollider = nullptr;
	}
	virtual ~Shape() {}
	virtual glm::vec3 GetHalfExtents() = 0;
	virtual glm::mat4& DebugModelMatrix() { glm::mat4 mat = glm::mat4(1.0f); return mat; }
	virtual bool TestPoint(float PointX, float PointY) = 0;

	Collider* mpOwnerCollider;

public:
	ShapeType mType;
};

class ShapeCircle : public Shape
{
public:
	ShapeCircle(float radius) : Shape(CIRCLE)
	{
		mRadius = radius;
	}
	bool TestPoint(float PointX, float PointY)
	{
		return false;
	}
	glm::vec3 GetHalfExtents() {
		glm::vec3 extents;
		extents = glm::vec3(mRadius, mRadius, mRadius);
		return extents;
	}

	float mRadius;
};

class ShapeAABB : public Shape
{
public:
	ShapeAABB(glm::vec3 min, glm::vec3 max) : Shape(AABB)
	{
		mMin = min;
		mMax = max;
	}
	bool TestPoint(float PointX, float PointY)
	{
		return false;
	}
	glm::vec3 GetHalfExtents() {
		glm::vec3 extents;
		extents = glm::vec3((mMax.x - mMin.x)*0.5f, (mMax.y - mMin.y)*0.5f, (mMax.z - mMin.z) * 0.5f);
		return extents;
	}

	glm::mat4& DebugModelMatrix() {
		mDebugMatrix = glm::translate(glm::mat4(1.0f), GetCenter());
		mDebugMatrix = glm::scale(mDebugMatrix, GetHalfExtents() * 2.0f);
		return mDebugMatrix;
	}

	bool Collides(ShapeAABB* other) {

		if ((mMin.x >= other->mMax.x || mMax.x <= other->mMin.x) ||
			(mMin.y >= other->mMax.y || mMax.y <= other->mMin.y) ||
			(mMin.z >= other->mMax.z || mMax.z <= other->mMin.z)
			) 
		{
			return false;
		}
		else {
			return true;
		}
	}

	// set the bounds of this AABB to encompass the argument AABBs
	void Merge(ShapeAABB& first, ShapeAABB& second) {
		mMin.x = std::min(first.mMin.x, second.mMin.x);
		mMin.y = std::min(first.mMin.y, second.mMin.y);
		mMin.z = std::min(first.mMin.z, second.mMin.z);

		mMax.x = std::max(first.mMax.x, second.mMax.x);
		mMax.y = std::max(first.mMax.y, second.mMax.y);
		mMax.z = std::max(first.mMax.z, second.mMax.z);
	}

	bool Contains(ShapeAABB& other) {
		if (other.mMin.x < mMin.x || other.mMin.y < mMin.y || other.mMin.z < mMin.z) {
			return false;
		}
		if (other.mMax.x > mMax.x || other.mMax.y > mMax.y || other.mMax.z > mMax.z) {
			return false;
		}
		return true;
	}

	glm::vec3 GetCenter() {
		return glm::vec3((mMax.x + mMin.x) * 0.5f, (mMax.y + mMin.y) *0.5f, (mMax.z + mMin.z) *0.5f);
	}

	float GetSurfaceArea() {
		glm::vec3 extents = GetHalfExtents();
		return 8.0f*(extents.x * extents.y + extents.y*extents.z + extents.z * extents.x);
	}

	glm::vec3 mMin, mMax;
	glm::mat4 mDebugMatrix;
};

