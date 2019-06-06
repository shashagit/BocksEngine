#pragma once

#include "../Components/Collider.h"
#include <vector>

struct FaceQuery {
	int faceIndex;
	float separation;
};

struct EdgeQuery {
	int edgeA;
	int edgeB;
	float separation;
};

class SAT
{
public:
	SAT();
	~SAT();

	//helper functions for Intersection Test
	FaceQuery FaceIntersectionQuery(Collider* col1, Collider* col2);
	EdgeQuery EdgeIntersectionQuery(Collider* col1, Collider* col2);

	bool TestIntersection3D(Collider* col1, Collider* col2);

};

