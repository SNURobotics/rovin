/**
*	\file	Constraint.cpp
*	\date	2015.11.07
*	\author	Keunjun (ckj@robotics.snu.ac.kr)
*	\brief	Constraint 클래스 구현
*/

#include "Constraint.h"

#include <cassert>
#include <Eigen/Dense>

using namespace Eigen;

namespace rovin
{
	namespace Math
	{
		Eigen::MatrixXd Constraint::Jacobian(const Eigen::VectorXd& q)
		{
			assert(q.size() == _NUM_VARIABLES);

			MatrixXd J(_NUM_CONSTRAINTS, _NUM_VARIABLES);
			VectorXd dq(_NUM_VARIABLES);
			J.setZero();
			dq.setZero();
			for (unsigned int i = 0; i < _NUM_VARIABLES; i++)
			{
				dq(i) = _DERIVATIVE_STEP_SIZE;

				J.col(i) = (func(q + dq) - func(q - dq)) / (2 * _DERIVATIVE_STEP_SIZE);

				dq(i) = 0.0;
			}

			return J;
		}

		Eigen::VectorXd Constraint::sovleEq(const Eigen::VectorXd& initial_guess)
		{
			assert(initial_guess.size() == _NUM_VARIABLES);

			Eigen::VectorXd q = initial_guess;
			Eigen::VectorXd fval = func(q);
			assert(fval.size() == _NUM_CONSTRAINTS);

			while (fval.norm() > NumTraits<double>::dummy_precision())
			{
				q = q - Jacobian(q).inverse()*fval;
				fval = func(q);
			}

			return q;
		}
	}
}