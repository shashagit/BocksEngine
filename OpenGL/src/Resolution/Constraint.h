#pragma once

#include <Eigen/Dense>
#include "../Components/Body.h"
#include "../NarrowPhase/Contact.h"



class Constraint
{
	Eigen::Matrix<float, 12, 1> deltaV;
public:
	Eigen::Matrix<float, 12, 12> massMatrixInverse;
	Eigen::Matrix<float, 1, 12> jacobian;
	Eigen::Matrix<float, 12, 1> jacobianT;
	Eigen::Matrix<float, 12, 1> velocityMatrix;
	// makes the jacobian for the given constraint
	inline void EvaluateJacobian(Contact* c, glm::vec3 dir)
	{
		jacobian(0, 0) = -dir.x;
		jacobian(0, 1) = -dir.y;
		jacobian(0, 2) = -dir.z;

		glm::vec3 rA_x_n = glm::cross(c->rA, dir);
		jacobian(0, 3) = -rA_x_n.x;
		jacobian(0, 4) = -rA_x_n.y;
		jacobian(0, 5) = -rA_x_n.z;

		jacobian(0, 6) = dir.x;
		jacobian(0, 7) = dir.y;
		jacobian(0, 8) = dir.z;

		glm::vec3 rB_x_n = glm::cross(c->rB, dir);
		jacobian(0, 9) = rB_x_n.x;
		jacobian(0, 10) = rB_x_n.y;
		jacobian(0, 11) = rB_x_n.z;

		
		jacobianT( 0, 0) = -dir.x;
		jacobianT(1, 0) = -dir.y;
		jacobianT(2, 0) = -dir.z;
		jacobianT(3, 0) = -rA_x_n.x;
		jacobianT(4, 0) = -rA_x_n.y;
		jacobianT(5, 0) = -rA_x_n.z;
		jacobianT(6, 0) = dir.x;
		jacobianT(7, 0) = dir.y;
		jacobianT(8, 0) = dir.z;
		jacobianT(9, 0) = rB_x_n.x;
		jacobianT(10, 0) = rB_x_n.y;
		jacobianT(11, 0) = rB_x_n.z;
	}

	// makes the mass matrix and calculates the effective mass
	inline void CalculateMassMatrixInv(Body* bodyA, Body* bodyB)
	{

		for (int i = 0; i < 12; ++i) {
			for (int j = 0; j < 12; ++j) {
				massMatrixInverse(i, j) = 0;
			}
		}

		massMatrixInverse(0, 0) = bodyA->mInvMass;
		massMatrixInverse(1, 1) = bodyA->mInvMass;
		massMatrixInverse(2, 2) = bodyA->mInvMass;

		for (int i = 3, m = 0; i < 6; ++i, ++m) {
			for (int j = 3, n = 0; j < 6; ++j, ++n) {
				massMatrixInverse(i, j) = bodyA->mInertiaWorldInverse[n][m];
			}
		}

		massMatrixInverse(6, 6) = bodyB->mInvMass;
		massMatrixInverse(7, 7) = bodyB->mInvMass;
		massMatrixInverse(8, 8) = bodyB->mInvMass;

		for (int i = 9, m = 0; i < 12; ++i, ++m) {
			for (int j = 9, n = 0; j < 12; ++j, ++n) {
				massMatrixInverse(i, j) = bodyB->mInertiaWorldInverse[n][m];
			}
		}
	}

	inline void EvaluateVelocityVector(Body* bodyA, Body* bodyB)
	{
		velocityMatrix(0, 0) = bodyA->mVel.x;
		velocityMatrix(1, 0) = bodyA->mVel.y;
		velocityMatrix(2, 0) = bodyA->mVel.z;

		velocityMatrix(3, 0) = bodyA->mAngularVel.x;
		velocityMatrix(4, 0) = bodyA->mAngularVel.y;
		velocityMatrix(5, 0) = bodyA->mAngularVel.z;

		velocityMatrix(6, 0) = bodyB->mVel.x;
		velocityMatrix(7, 0) = bodyB->mVel.y;
		velocityMatrix(8, 0) = bodyB->mVel.z;

		velocityMatrix(9, 0) = bodyB->mAngularVel.x;
		velocityMatrix(10, 0) = bodyB->mAngularVel.y;
		velocityMatrix(11, 0) = bodyB->mAngularVel.z;
	}

	inline void ApplyImpulse(Body* bodyA, Body* bodyB, float impulse)
	{
		deltaV = massMatrixInverse * jacobianT * impulse;

		bodyA->mVel += glm::vec3(deltaV(0, 0), deltaV(1, 0), deltaV(2, 0));
		bodyA->mAngularVel += glm::vec3(deltaV(3, 0), deltaV(4, 0), deltaV(5, 0));

		bodyB->mVel += glm::vec3(deltaV(6, 0), deltaV(7, 0), deltaV(8, 0));
		bodyB->mAngularVel += glm::vec3(deltaV(9, 0), deltaV(10, 0), deltaV(11, 0));
	}

};

