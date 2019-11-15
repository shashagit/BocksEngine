#include "SAT.h"
#include "../Components/Collider.h"
#include "../Components/Body.h"
#include "../Components/Shape.h"

#include "../Managers/CollisionManager.h"

#include <assert.h>

#include <glm/gtc/matrix_access.hpp>
#include"imgui/imgui.h"
#include"imgui/imgui_impl_glfw_gl3.h"

extern CollisionManager* colMan;

#define epsilon 0.0001f

SAT::SAT()
{
	
}


SAT::~SAT()
{
}

// from http://paulbourke.net/geometry/pointlineplane/
float LineLineIntersect(
	glm::vec3 p1, glm::vec3 p2, glm::vec3 p3, glm::vec3 p4, glm::vec3* poA, glm::vec3* poB)
{
	glm::vec3 p13, p43, p21, pa, pb;
	double d1343, d4321, d1321, d4343, d2121;
	double numer, denom;

	p13.x = p1.x - p3.x;
	p13.y = p1.y - p3.y;
	p13.z = p1.z - p3.z;
	p43.x = p4.x - p3.x;
	p43.y = p4.y - p3.y;
	p43.z = p4.z - p3.z;
	
	p21.x = p2.x - p1.x;
	p21.y = p2.y - p1.y;
	p21.z = p2.z - p1.z;

	d1343 = p13.x * p43.x + p13.y * p43.y + p13.z * p43.z;
	d4321 = p43.x * p21.x + p43.y * p21.y + p43.z * p21.z;
	d1321 = p13.x * p21.x + p13.y * p21.y + p13.z * p21.z;
	d4343 = p43.x * p43.x + p43.y * p43.y + p43.z * p43.z;
	d2121 = p21.x * p21.x + p21.y * p21.y + p21.z * p21.z;

	denom = d2121 * d4343 - d4321 * d4321;
	if (abs(denom) < 0.0001)
		return 0.0f;
	numer = d1343 * d4321 - d1321 * d4343;

	double mua = numer / denom;
	double mub = (d1343 + d4321 * (mua)) / d4343;

	pa.x = p1.x + mua * p21.x;
	pa.y = p1.y + mua * p21.y;
	pa.z = p1.z + mua * p21.z;
	pb.x = p3.x + mub * p43.x;
	pb.y = p3.y + mub * p43.y;
	pb.z = p3.z + mub * p43.z;

	*poA = pa;
	*poB = pb;

	return glm::distance(pa, pb);
}

// from Dirk Gregorius' Post on GameDev.net
std::vector<glm::vec3> ClipPolygon(const std::vector<glm::vec3>& polygon, glm::vec3 normal, glm::vec3 pointOnPlane) {
	std::vector<glm::vec3> clipped;

	glm::vec3 ver1 = polygon.back();

	float dist1 = glm::dot(normal, ver1 - pointOnPlane);

	for (int i = 0; i < polygon.size(); ++i) {
		glm::vec3 ver2 = polygon[i];
		float dist2 = glm::dot(normal, ver2 - pointOnPlane);

		if (dist1 <= 0.0f and dist2 <= 0.0f) {
			clipped.push_back(ver2);
		}
		else if (dist1 <= 0.0f and dist2 > 0.0f) {
			float frac = dist1 / (dist1 - dist2);
			glm::vec3 intersectionPoint = ver1 + frac * (ver2 - ver1);

			clipped.push_back(intersectionPoint);
		}
		else if (dist2 <= 0.0f and dist1 > 0.0f) {
			float frac = dist1 / (dist1 - dist2);
			glm::vec3 intersectionPoint = ver1 + frac * (ver2 - ver1);

			clipped.push_back(intersectionPoint);
			clipped.push_back(ver2);
		}

		ver1 = ver2;
		dist1 = dist2;
	}

	return clipped;
}

// SAT Gregorius style
bool SAT::TestIntersection3D(Collider* col1, Collider* col2) {
	FaceQuery faceQueryA = FaceIntersectionQuery(col1, col2);
	if (faceQueryA.separation > 0.0f) return false;

	FaceQuery faceQueryB = FaceIntersectionQuery(col2, col1);
	if (faceQueryB.separation > 0.0f) return false;

	EdgeQuery edgeQuery = EdgeIntersectionQuery(col1, col2);
	if (edgeQuery.separation > 0.0f) return false;

	bool refFaceOnA = false;
	bool refFaceOnB = false;
	Collider* refCollider = nullptr;
	Collider* inciCollider = nullptr;
	int refIndex = -1;

	ContactManifold* manifold = new ContactManifold();

	if (std::min(fabs(faceQueryA.separation), fabs(faceQueryB.separation)) < fabs(edgeQuery.separation) + epsilon) {
		// Find Axis of Minimum Penetration
		if (fabs(faceQueryA.separation) < fabs(faceQueryB.separation) + epsilon) {
			// faceQueryA face index is the reference face
			refFaceOnA = true;
			refCollider = col1;
			inciCollider = col2;
			refIndex = faceQueryA.faceIndex;
		}
		else if (fabs(faceQueryB.separation) < fabs(faceQueryA.separation) + epsilon) {
			refFaceOnB = true;
			refCollider = col2;
			inciCollider = col1;
			refIndex = faceQueryB.faceIndex;
		}
	}

	// if Minimum Axis of Penetration is Face Normal
	if (refFaceOnA || refFaceOnB) {
		glm::vec3 refFaceNormal = glm::vec3(0.0f);
		MeshData& incidentMeshData = inciCollider->meshData;
		MeshData& referenceMeshData = refCollider->meshData;

		refFaceNormal = refCollider->mpBody->mRotationMatrix * refCollider->meshData.faces[refIndex].normal;

		// Find incident face
		int incidentIndex = -1;
		float minProjection = std::numeric_limits<float>::infinity();

		for (int i = 0; i < incidentMeshData.faces.size(); ++i) {
			glm::vec3 normalWorld = inciCollider->mpBody->mRotationMatrix * incidentMeshData.faces[i].normal;

			// find most anti-parallel face on the body
			float projection = glm::dot(refFaceNormal, normalWorld);
			if (minProjection > projection) {
				minProjection = projection;
				incidentIndex = i;
			}
		}

		// Get Incident Face Vertices
		std::vector<glm::vec3> incidentPoly = incidentMeshData.GetFacePolygon(incidentIndex);
		for (auto& v : incidentPoly) {
			v = inciCollider->mpBody->mRotationMatrix * v + inciCollider->mpBody->mPos;
		}

		std::vector<glm::vec3>& clippedPoly = incidentPoly;
		int startingEdge = referenceMeshData.faces[refIndex].edge;

		// clip against adjacent faces to the reference face
		for (int i = 0; i < 4; ++i) {
			int twin = referenceMeshData.edges[startingEdge].twin;

			glm::vec3 pointOnFace = referenceMeshData.GetPointOnFace(referenceMeshData.edges[twin].face);
			glm::vec3 faceNormal = referenceMeshData.faces[referenceMeshData.edges[twin].face].normal;

			pointOnFace = refCollider->mpBody->mRotationMatrix * pointOnFace + refCollider->mpBody->mPos;
			faceNormal = refCollider->mpBody->mRotationMatrix * faceNormal;

			if(!clippedPoly.empty())
				clippedPoly = ClipPolygon(clippedPoly, faceNormal, pointOnFace);

			startingEdge = referenceMeshData.edges[startingEdge].next;
		}

		// clip against reference face
		glm::vec3 pointOnRef = refCollider->mpBody->mRotationMatrix * referenceMeshData.GetPointOnFace(refIndex)
			+ refCollider->mpBody->mPos ;
		
		if (!clippedPoly.empty())
			clippedPoly = ClipPolygon(clippedPoly, refFaceNormal, pointOnRef);

		Contact deepest;
		deepest.penetrationDepth = std::numeric_limits<float>::max();

		std::vector<Contact> tempContacts;
		
		// save penetration depth for all points from the reference face
		for (auto& point : clippedPoly) {
			float depth = glm::dot(refFaceNormal, point - pointOnRef);
			if (depth <= 0.0f) {
				Contact c;
				c.penetrationDepth = depth;
				c.point = point;

				tempContacts.emplace_back(c);

				if(deepest.penetrationDepth > depth)
				{
					deepest = c;
				}
			}
		}
		
		manifold->contactPoints.emplace_back(deepest);
				
		if(tempContacts.size() <= 4)
		{
			manifold->contactPoints.swap(tempContacts);
		}
		else 
		{
			// Select 3 more points out of all available
			Contact furthest, triangle, fourth;
			float far = -std::numeric_limits<float>::max();
			for (auto& point : tempContacts)
			{
				float distSq = glm::distance2(point.point, deepest.point);
				if(distSq > far)
				{
					far = distSq;
					furthest = point;
				}
			}
			manifold->contactPoints.emplace_back(furthest);

			// find largest area triangle
			float maxArea = -std::numeric_limits<float>::max();
			int winding = 0;
			for (auto& point : tempContacts)
			{
				glm::vec3 side1 = manifold->contactPoints[0].point - point.point;
				glm::vec3 side2 = manifold->contactPoints[1].point - point.point;

				float area = 0.5f * glm::dot(glm::cross(side1, side2), refFaceNormal);
				if (abs(area) > maxArea)
				{
					if (area < 0) winding = -1; else winding = 1;
					maxArea = abs(area);
					triangle = point;
				}
			}
			manifold->contactPoints.emplace_back(triangle);

			maxArea = -std::numeric_limits<float>::max();
			glm::vec3 side1(0);
			glm::vec3 side2(0);
			int A = 0, B = 1, C = 2;
			if(winding < 0)
			{
				A = 2; B = 0; C = 1;
			}
			float totalArea = 0.0f, area;
			for (auto& point : tempContacts)
			{
				totalArea = 0.0f;
				
				side1 = manifold->contactPoints[A].point - point.point;
				side2 = manifold->contactPoints[B].point - point.point;

				area = 0.5f * glm::dot(glm::cross(side1, side2), refFaceNormal);
				if (area < 0) 
				totalArea += area;
				
				side1 = manifold->contactPoints[B].point - point.point;
				side2 = manifold->contactPoints[C].point - point.point;

				area = 0.5f * glm::dot(glm::cross(side1, side2), refFaceNormal);
				if (area < 0)
				totalArea += area;

				side1 = manifold->contactPoints[C].point - point.point;
				side2 = manifold->contactPoints[A].point - point.point;

				area = 0.5f * glm::dot(glm::cross(side1, side2), refFaceNormal);
				if (area < 0)
				totalArea += area;

				if (abs(totalArea) > maxArea)
				{
					maxArea = abs(totalArea);
					fourth = point;
				}
			}
			manifold->contactPoints.emplace_back(fourth);
			
		}
		
		
		// project points onto reference face to calculate rA and rB
		int size = manifold->contactPoints.size();
		for (int i = 0; i < size; ++i) {
			glm::vec3 projected = manifold->contactPoints[i].point - refFaceNormal * manifold->contactPoints[i].penetrationDepth;
			
			manifold->contactPoints[i].rA = projected - refCollider->mpBody->mPos;
			manifold->contactPoints[i].rB = manifold->contactPoints[i].point - inciCollider->mpBody->mPos;
		}

		// save contact normal
		manifold->collisionNormal = refFaceNormal;

		// for consistent normal orientation
		if (glm::dot(manifold->collisionNormal, inciCollider->mpBody->mPos - refCollider->mpBody->mPos) < 0.0f) {
			manifold->collisionNormal = -manifold->collisionNormal;
		}

		manifold->bodyA = refCollider->mpBody;
		manifold->bodyB = inciCollider->mpBody;

		// push back the contacts into the list of manifolds
		colMan->mContacts->push_back(manifold);
	}
	else if(edgeQuery.edgeA != -1) {
	// edge case
		glm::vec3 pA1 = col1->meshData.vertices[col1->meshData.edges[col1->meshData.edges[edgeQuery.edgeA].prev].toVertex].point;
		glm::vec3 pA2 = col1->meshData.vertices[col1->meshData.edges[edgeQuery.edgeA].toVertex].point;

		pA1 = col1->mpBody->mRotationMatrix * pA1 + col1->mpBody->mPos;
		pA2 = col1->mpBody->mRotationMatrix * pA2 + col1->mpBody->mPos;

		glm::vec3 pB1 = col2->meshData.vertices[col2->meshData.edges[col2->meshData.edges[edgeQuery.edgeB].prev].toVertex].point;
		glm::vec3 pB2 = col2->meshData.vertices[col2->meshData.edges[edgeQuery.edgeB].toVertex].point;
		
		pB1 = col2->mpBody->mRotationMatrix * pB1 + col2->mpBody->mPos;
		pB2 = col2->mpBody->mRotationMatrix * pB2 + col2->mpBody->mPos;
		
		// find the point betweem the edges
		glm::vec3 pointOnA, pointOnB;
		float depth	= LineLineIntersect(pA1, pA2, pB1, pB2, &pointOnA, &pointOnB);

		glm::vec3 edgeA = pA2 - pA1;
		glm::vec3 edgeB = pB2 - pB1;

		Contact c;
		c.point = 0.5f * (pointOnA + pointOnB);
		c.penetrationDepth = -depth;

		c.rA = pointOnA - col1->mpBody->mPos;
		c.rB = pointOnB - col2->mpBody->mPos;

		manifold->collisionNormal = glm::cross(edgeA, edgeB);

		// for consistent normal orientation
		if (glm::dot(manifold->collisionNormal, col2->mpBody->mPos - col1->mpBody->mPos) < 0.0f) {
			manifold->collisionNormal = -manifold->collisionNormal;
		}

		//c->rA = poC - col1->mpBody->mPos;
		//c->rB = poC - col2->mpBody->mPos;

		//manifold->collisionNormal = glm::cross(col1->mpBody->mRotationMatrix * col1->meshData.GetEdgeDirection(edgeQuery.edgeA),
			//col2->mpBody->mRotationMatrix * col2->meshData.GetEdgeDirection(edgeQuery.edgeB));

		manifold->bodyA = col1->mpBody;
		manifold->bodyB = col2->mpBody;

		manifold->contactPoints.emplace_back(c);

		colMan->mContacts->push_back(manifold);
	}

	//{
	//	//bool open = true;
	//	//ImGui::SetNextWindowPos(ImVec2(1096, 0));
	//	ImGui::Begin("SAT");// , &open, ImVec2(344, 105), 0.9f, ImGuiWindowFlags_NoResize); // Create a window called "Hello, world!" and append into it.
	//	//ImGui::SetWindowSize(ImVec2(344, 105));

	//	ImGui::Text("separationA = %f", faceQueryA.separation);
	//	ImGui::Text("separationB = %f", faceQueryB.separation);
	//	ImGui::Text("separationE = %f", edgeQuery.separation);
	//	ImGui::Text("axisIdA = %d", faceQueryA.faceIndex);
	//	ImGui::Text("axisIdB = %d", faceQueryB.faceIndex);
	//	ImGui::Text("axisIdE1 = %d", edgeQuery.edgeA);
	//	ImGui::Text("axisIdE2 = %d", edgeQuery.edgeB);
	//	//ImGui::Text("axisId = %f %f %f", colNorm.x, colNorm.y, colNorm.z);
	//	//ImGui::Text("clipPoly = %d", clip);

	//	//ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
	//	//ImGui::Checkbox("Debug Draw", &mRenderSystem->debug_Select);

	//	ImGui::End();
	//}

	// setting up mass matrix inv here
	manifold->SetupGroundConstraint();
	
	return true;
}


FaceQuery SAT::FaceIntersectionQuery(Collider* col1, Collider* col2) {
	glm::mat3 Ra = col1->mpBody->mRotationMatrix;
	glm::mat3 Rb = col2->mpBody->mRotationMatrix;

	// rotation matrix to convert from A's local to B's local
	//glm::mat3 C = Rb * glm::transpose(Ra);
	glm::mat3 C = glm::transpose(Rb) * Ra;

	FaceQuery fq;
	fq.faceIndex = -1;
	fq.separation = -std::numeric_limits<float>::infinity();

	for (int i = 0; i < col1->meshData.faces.size(); ++i) {
		glm::vec3 normalInBSpace = C * col1->meshData.faces[i].normal;
		
		glm::vec3 facePointinBSpace = C * (col1->meshData.GetPointOnFace(i)) + glm::transpose(Rb) * (col1->mpBody->mPos - col2->mpBody->mPos);

		glm::vec3 supportPoint = col2->meshData.GetSupport(-normalInBSpace);

		float s = glm::dot(normalInBSpace, supportPoint - facePointinBSpace);
		if (s > fq.separation) {
			fq.separation = s;
			fq.faceIndex = i;
		}
	}

	return fq;
}

// Gauss Map Optimisation
bool isMinkowskiFace(const glm::vec3& A, const glm::vec3& B, const glm::vec3& B_x_A, const glm::vec3& C, const glm::vec3& D, const glm::vec3& D_x_C)
{
	// Test if arcs AB and CD intersect on the unit sphere 
	float CBA = glm::dot(C, B_x_A);
	float DBA = glm::dot(D, B_x_A);
	float ADC = glm::dot(A, D_x_C);
	float BDC = glm::dot(B, D_x_C);

	return CBA * DBA < 0.0f && ADC * BDC < 0.0f && CBA * BDC > 0.0f;
}


float Project(glm::vec3 p1, glm::vec3 edge1, glm::vec3 p2, glm::vec3 edge2, glm::vec3 center) {
	const float kTol = 0.005f;
	glm::vec3 edge1Xedge2 = glm::cross(edge1, edge2);
	float L = glm::length(edge1Xedge2);
	if (L < kTol * sqrtf(glm::length2(edge1) * glm::length2(edge2))) {
		return -std::numeric_limits<float>::infinity();
	}

	glm::vec3 norm = edge1Xedge2 / L;
	if (glm::dot(norm, p1 - center) < 0.0f) {
		norm = -norm;
	}

	return glm::dot(norm, p2 - p1);
}

EdgeQuery SAT::EdgeIntersectionQuery(Collider* col1, Collider* col2) {
	glm::mat3 Ra = col1->mpBody->mRotationMatrix;
	glm::mat3 Rb = col2->mpBody->mRotationMatrix;

	// rotation matrix to convert from A's local to B's local
	glm::mat3 C = glm::transpose(Rb) * Ra;

	glm::vec3 centerA = glm::transpose(Rb) * (col1->mpBody->mPos - col2->mpBody->mPos);

	EdgeQuery eq;
	eq.edgeA = -1;
	eq.edgeB = -1;
	eq.separation = -std::numeric_limits<float>::infinity();

	for (int i = 0; i < col1->meshData.edges.size(); i += 2) {
		glm::vec3 edge1Dir = C * col1->meshData.GetEdgeDirection(i);
		glm::vec3 p1 = C * col1->meshData.vertices[col1->meshData.edges[col1->meshData.edges[i].prev].toVertex].point + centerA;
		
		glm::vec3 u1 = C * col1->meshData.faces[col1->meshData.edges[i].face].normal;
		glm::vec3 v1 = C * col1->meshData.faces[col1->meshData.edges[i+1].face].normal;

		assert(i + 1 == col1->meshData.edges[i].twin);

		for (int j = 0; j < col2->meshData.edges.size(); j += 2) {
			glm::vec3 edge2Dir = col2->meshData.GetEdgeDirection(j);
			glm::vec3 p2 = col2->meshData.vertices[col2->meshData.edges[col2->meshData.edges[j].prev].toVertex].point;

			glm::vec3 u2 = col2->meshData.faces[col2->meshData.edges[j].face].normal;
			glm::vec3 v2 = col2->meshData.faces[col2->meshData.edges[j + 1].face].normal;

			// check arcs intersection on gauss map to cull edge checks
			if (isMinkowskiFace(u1, v1, -edge1Dir, -u2, -v2, -edge2Dir)) {
				float s = Project(p1, edge1Dir, p2, edge2Dir, centerA);
				if (s > eq.separation) {
					eq.separation = s;
					eq.edgeA = i;
					eq.edgeB = j;
				}
			}
		}
	}

	return eq;
}

// OBB-OBB test optimised
//bool FindAxisMinimumPenetration(int id, int* axisId, float s, float* separation, const glm::vec3& normal, glm::vec3* axisNormal, bool isEdge) {
//	if (s > 0.0f)
//		return true;
//
//	if (!isEdge) {
//		if (s > *separation) 
//			*separation = s;
//			*axisNormal = normal;
//			*axisId = id;
//		}
//	}
//	else {
//		float l = 1.0f / glm::length(normal);
//		s *= l;
//
//		if (s > *separation) {
//			*separation = s;
//			*axisNormal = normal * l;
//			*axisId = id;
//		}
//	}
//
//	return false;
//}
//bool SAT::TestIntersection3D(Collider* col1, Collider* col2)
//{
//	glm::mat3 Ra = col1->mpBody->mRotationMatrix;
//	glm::mat3 Rb = col2->mpBody->mRotationMatrix;
//
//	glm::vec3 eA = col1->mpLocalShape->GetHalfExtents();
//	glm::vec3 eB = col2->mpLocalShape->GetHalfExtents();
//
//	// rotation matrix to convert from B's local to A's local
//	glm::mat3 C = glm::transpose(Rb) * Ra;
//
//	// calculate absolute of C and find if parallel axes exist
//	glm::mat3 absC;
//	bool parallel = false;
//	float EPSILON = 1.0e-6;
//	for (int i = 0; i < 3; ++i) {
//		for (int j = 0; j < 3; ++j) {
//			float val = fabsf(C[i][j]);
//			//absC[i][j] = val + EPSILON;
//			absC[i][j] = val;
//
//			if (val + EPSILON > 1.0f) {
//				parallel = true;
//			}
//		}
//	}
//
//	// center difference vector transformed to A's local space
//	glm::vec3 t = Ra * (col2->mpBody->mPos - col1->mpBody->mPos);
//
//	float separation = 0.0f;
//
//	float maxSeparation = -std::numeric_limits<float>::infinity();
//	glm::vec3 colNorm = glm::vec3(0.0f);
//	int axisId = -1;
//
//	// ========ERICSON========
//	//float ra, rb;
//	//float ea[3], eb[3];
//	//ea[0] = eA.x;
//	//ea[1] = eA.y;
//	//ea[2] = eA.z;
//	//eb[0] = eB.x;
//	//eb[1] = eB.y;
//	//eb[2] = eB.z;
//
//	//for (int i = 0; i < 3; i++) {
//	//	ra = ea[i];
//	//	rb = eb[0] * absC[i][0] + eb[1] * absC[i][1] + eb[2] * absC[i][2];
//	//	if (fabs(t[i]) > ra + rb) return false;
//	//}
//	//// Test axes L = B0, L = B1, L = B2
//	//for (int i = 0; i < 3; i++) {
//	//	ra = ea[0] * absC[0][i] + ea[1] * absC[1][i] + ea[2] * absC[2][i];
//	//	rb = eb[i];
//	//	if (fabs(t[0] * C[0][i] + t[1] * C[1][i] + t[2] * C[2][i]) > ra + rb) return false;
//	//}
//
//
//	// ========RANDY========
//	// A's local x ie. world X axis
//	separation = fabsf(t.x) - (eA.x + glm::dot(glm::column(absC, 0), eB));
//	if (FindAxisMinimumPenetration(0, &axisId, separation, &maxSeparation, glm::row(Ra, 0), &colNorm, false))
//		return false;
//
//	// A's local y ie. world Y axis
//	separation = fabsf(t.y) - (eA.y + glm::dot(glm::column(absC, 1), eB));
//	if (FindAxisMinimumPenetration(1, &axisId, separation, &maxSeparation, glm::row(Ra, 1), &colNorm, false))
//		return false;
//
//	// A's local z ie. world Z axis
//	separation = fabsf(t.z) - (eA.z + glm::dot(glm::column(absC, 2), eB));
//	if (FindAxisMinimumPenetration(2, &axisId, separation, &maxSeparation, glm::row(Ra, 2), &colNorm, false))
//		return false;
//
//	// B's local x ie. 0th row from C matrix
//	separation = fabsf(glm::dot(t, glm::row(C, 0))) - (eB.x + glm::dot(glm::row(absC, 0), eA));
//	if (FindAxisMinimumPenetration(3, &axisId, separation, &maxSeparation, glm::row(Rb, 0), &colNorm, false))
//		return false;
//
//	// B's local y ie. 1st row from C matrix
//	separation = fabsf(glm::dot(t, glm::row(C, 1))) - (eB.y + glm::dot(glm::row(absC, 1), eA));
//	if (FindAxisMinimumPenetration(4, &axisId, separation, &maxSeparation, glm::row(Rb, 1), &colNorm, false))
//		return false;
//
//	// B's local z ie. 2nd row from C matrix
//	separation = fabsf(glm::dot(t, glm::row(C, 2))) - (eB.z + glm::dot(glm::row(absC, 2), eA));
//	if (FindAxisMinimumPenetration(5, &axisId, separation, &maxSeparation, glm::row(Rb, 2), &colNorm, false))
//		return false;
//
//	if(!parallel)
//	{
//		// Edge axis checks
//		float rA;
//		float rB;
//
//		// Cross( a.x, b.x )
//		rA = eA.y * absC[0][2] + eA.z * absC[0][1];
//		rB = eB.y * absC[2][0] + eB.z * absC[1][0];
//		separation = fabsf(t.z * C[0][1] - t.y * C[0][2]) - (rA + rB);
//		if (FindAxisMinimumPenetration(6, &axisId, separation, &maxSeparation, glm::vec3(0.0f, -C[0][2], C[0][1]), &colNorm, true))
//			return false;
//
//		// Cross( a.x, b.y )
//		rA = eA.y * absC[1][2] + eA.z * absC[1][1];
//		rB = eB.x * absC[2][0] + eB.z * absC[0][0];
//		separation = fabsf(t.z * C[1][1] - t.y * C[1][2]) - (rA + rB);
//		if (FindAxisMinimumPenetration(7, &axisId, separation, &maxSeparation, glm::vec3(0.0f, -C[1][2], C[1][1]), &colNorm, true))
//			return false;
//
//		// Cross( a.x, b.z )
//		rA = eA.y * absC[2][2] + eA.z * absC[2][1];
//		rB = eB.x * absC[1][0] + eB.y * absC[0][0];
//		separation = fabsf(t.z * C[2][1] - t.y * C[2][2]) - (rA + rB);
//		if (FindAxisMinimumPenetration(8, &axisId, separation, &maxSeparation, glm::vec3(0.0f, -C[2][2], C[2][1]), &colNorm, true))
//			return false;
//
//		// Cross( a.y, b.x )
//		rA = eA.x * absC[0][2] + eA.z * absC[0][0];
//		rB = eB.y * absC[2][1] + eB.z * absC[1][1];
//		separation = fabsf(t.x * C[0][2] - t.z * C[0][0]) - (rA + rB);
//		if (FindAxisMinimumPenetration(9, &axisId, separation, &maxSeparation, glm::vec3(C[0][2], 0.0f, -C[0][0]), &colNorm, true))
//			return false;
//
//		// Cross( a.y, b.y )
//		rA = eA.x * absC[1][2] + eA.z * absC[1][0];
//		rB = eB.x * absC[2][1] + eB.z * absC[0][1];
//		separation = fabsf(t.x * C[1][2] - t.z * C[1][0]) - (rA + rB);
//		if (FindAxisMinimumPenetration(10, &axisId, separation, &maxSeparation, glm::vec3(C[1][2], 0.0f, -C[1][0]), &colNorm, true))
//			return false;
//
//		// Cross( a.y, b.z )
//		rA = eA.x * absC[2][2] + eA.z * absC[2][0];
//		rB = eB.x * absC[1][1] + eB.y * absC[0][1];
//		separation = fabsf(t.x * C[2][2] - t.z * C[2][0]) - (rA + rB);
//		if (FindAxisMinimumPenetration(11, &axisId, separation, &maxSeparation, glm::vec3(C[2][2], 0.0f, -C[2][0]), &colNorm, true))
//			return false;
//
//		// Cross( a.z, b.x )
//		rA = eA.x * absC[0][1] + eA.y * absC[0][0];
//		rB = eB.y * absC[2][2] + eB.z * absC[1][2];
//		separation = fabsf(t.y * C[0][0] - t.x * C[0][1]) - (rA + rB);
//		if (FindAxisMinimumPenetration(12, &axisId, separation, &maxSeparation, glm::vec3(-C[0][1], C[0][0], 0.0f), &colNorm, true))
//			return false;
//
//		// Cross( a.z, b.y )
//		rA = eA.x * absC[1][1] + eA.y * absC[1][0];
//		rB = eB.x * absC[2][2] + eB.z * absC[0][2];
//		separation = fabsf(t.y * C[1][0] - t.x * C[1][1]) - (rA + rB);
//		if (FindAxisMinimumPenetration(13, &axisId, separation, &maxSeparation, glm::vec3(-C[1][1], C[1][0], 0.0f), &colNorm, true))
//			return false;
//
//		// Cross( a.z, b.z )
//		rA = eA.x * absC[2][1] + eA.y * absC[2][0];
//		rB = eB.x * absC[1][2] + eB.y * absC[0][2];
//		separation = fabsf(t.y * C[2][0] - t.x * C[2][1]) - (rA + rB);
//		if (FindAxisMinimumPenetration(14, &axisId, separation, &maxSeparation, glm::vec3(-C[2][1], C[2][0], 0.0f), &colNorm, true))
//			return false;
//	}
//
//	{
//		//bool open = true;
//		//ImGui::SetNextWindowPos(ImVec2(1096, 0));
//		ImGui::Begin("SAT");// , &open, ImVec2(344, 105), 0.9f, ImGuiWindowFlags_NoResize); // Create a window called "Hello, world!" and append into it.
//		//ImGui::SetWindowSize(ImVec2(344, 105));
//
//		ImGui::Text("separation = %f", maxSeparation);
//		ImGui::Text("axisId = %d", axisId);
//		ImGui::Text("axisId = %f %f %f", colNorm.x, colNorm.y, colNorm.z);
//		//ImGui::Text("physicsFrames = %d", physicsUpdates);
//
//		//ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
//		//ImGui::Checkbox("Debug Draw", &mRenderSystem->debug_Select);
//
//		ImGui::End();
//	}
//
//	// no separation was detected on all axes checked 
//	return true;
//}
