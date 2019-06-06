#pragma once

#include "../Managers/CollisionManager.h"
#include <Eigen/Dense>

class Constraint
{
public:
	Constraint();
	~Constraint();

	Eigen::Matrix<float, 12, 12> massMatrixInverse;
	Eigen::Matrix<float, 1, 12> jacobian;
	Eigen::Matrix<float, 12, 1> velocityMatrix;
	
	// makes the jacobian for the given constraint
	void EvaluateJacobian(const ContactManifold& contact, glm::vec3, int);

	// makes the mass matrix and calculates the effective mass
	void CalculateMassMatrixInv(const ContactManifold& contact);

	void EvaluateVelocityVector(const ContactManifold& contact);

};

