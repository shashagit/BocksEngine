#include "Constraint.h"



Constraint::Constraint()
{
}


Constraint::~Constraint()
{
}

void Constraint::EvaluateJacobian(const ContactManifold & contact, glm::vec3 dir, int cpIndex)
{

	jacobian(0, 0) = -dir.x;
	jacobian(0, 1) = -dir.y;
	jacobian(0, 2) = -dir.z;

	glm::vec3 rA_x_n = glm::cross(contact.contactPoints[cpIndex]->rA, dir);
	jacobian(0, 3) = -rA_x_n.x;
	jacobian(0, 4) = -rA_x_n.y;
	jacobian(0, 5) = -rA_x_n.z;
				 
	jacobian(0, 6) = dir.x;
	jacobian(0, 7) = dir.y;
	jacobian(0, 8) = dir.z;

	glm::vec3 rB_x_n = glm::cross(contact.contactPoints[cpIndex]->rB, dir);
	jacobian(0, 9) = rB_x_n.x;
	jacobian(0, 10) = rB_x_n.y;
	jacobian(0, 11) = rB_x_n.z;
}

void Constraint::CalculateMassMatrixInv(const ContactManifold & contact)
{

	for (int i = 0; i < 12; ++i) {
		for (int j = 0; j < 12; ++j) {
			massMatrixInverse(i, j) = 0;
		}
	}

	massMatrixInverse(0, 0) = contact.bodyA->mInvMass;
	massMatrixInverse(1, 1) = contact.bodyA->mInvMass;
	massMatrixInverse(2, 2) = contact.bodyA->mInvMass;

	for (int i = 3, m = 0; i < 6; ++i, ++m) {
		for (int j = 3, n = 0; j < 6; ++j, ++n) {
			massMatrixInverse(i, j) = contact.bodyA->mInertiaWorldInverse[n][m];
		}
	}

	massMatrixInverse(6, 6) = contact.bodyB->mInvMass;
	massMatrixInverse(7, 7) = contact.bodyB->mInvMass;
	massMatrixInverse(8, 8) = contact.bodyB->mInvMass;

	for (int i = 9, m = 0; i < 12; ++i, ++m) {
		for (int j = 9, n = 0; j < 12; ++j, ++n) {
			massMatrixInverse(i, j) = contact.bodyB->mInertiaWorldInverse[n][m];
		}
	}
}

void Constraint::EvaluateVelocityVector(const ContactManifold & contact)
{
	velocityMatrix(0, 0) = contact.bodyA->mVel.x;
	velocityMatrix(1, 0) = contact.bodyA->mVel.y;
	velocityMatrix(2, 0) = contact.bodyA->mVel.z;

	velocityMatrix(3, 0) = contact.bodyA->mAngularVel.x;
	velocityMatrix(4, 0) = contact.bodyA->mAngularVel.y;
	velocityMatrix(5, 0) = contact.bodyA->mAngularVel.z;

	velocityMatrix(6, 0) = contact.bodyB->mVel.x;
	velocityMatrix(7, 0) = contact.bodyB->mVel.y;
	velocityMatrix(8, 0) = contact.bodyB->mVel.z;

	velocityMatrix(9, 0) = contact.bodyB->mAngularVel.x;
	velocityMatrix(10, 0) = contact.bodyB->mAngularVel.y;
	velocityMatrix(11, 0) = contact.bodyB->mAngularVel.z;
}
