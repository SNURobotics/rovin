/**
*	\file	Constraint.h
*	\date	2015.11.07
*	\author	Keunjun (ckj@robotics.snu.ac.kr)
*	\brief	Constraint에 관련된 변수와 함수를 담고 있는 클래스
*/

#pragma once

#include <list>
#include <Eigen/Dense>

namespace rovin
{
	namespace Math
	{
		/**
		*	\class Constraint
		*	\brief func(q)=0 또는 func(q)<=0 문제를 q에 대해서 풀어주는 클래스입니다. 가상함수입니다.
		*/
		class Constraint
		{
		public:
			/// 생성자
			Constraint() : _NUM_VARIABLES(0), _NUM_CONSTRAINTS(0) {}
			/// 변수의 갯수와 Constraint의 갯수를 정해줍니다.
			Constraint(const unsigned int& n_variables ///< 변수의 갯수
				, const unsigned int& n_constraints ///< 조건의 갯수 (즉, func의 행의 갯수)
				) : _NUM_VARIABLES(n_variables), _NUM_CONSTRAINTS(n_constraints) {}

			/// 변수의 갯수를 설정합니다.
			void setNumVariables(const unsigned int& n_variables ///< 변수의 갯수
				)
			{
				_NUM_VARIABLES = n_variables;
			}
			/// 조건의 갯수를 설정합니다.
			void setNumConstraints(const unsigned int& n_constraints ///< 조건의 갯수
				)
			{
				_NUM_CONSTRAINTS = n_constraints;
			}

			/// 변수의 갯수를 가지고 옵니다.
			const unsigned int& getNumVariabales() const
			{
				return _NUM_VARIABLES;
			}
			/// 조건의 갯수를 가지고 옵니다.
			const unsigned int& getNumConstraints() const
			{
				return _NUM_CONSTRAINTS;
			}

			/// Constraint에 해당하는 식을 함수로 표현합니다. func(q)
			virtual Eigen::VectorXd func(const Eigen::VectorXd& q ///< 변수를 받습니다.
				) = 0;
			virtual Eigen::MatrixXd Jacobian(const Eigen::VectorXd& q);

			/// Newton-Raphson method를 이용해서 func(q) = 0 문제를 q에 대해서 푼다.
			Eigen::VectorXd sovleEq(const Eigen::VectorXd& initial_guess ///< 초기값을 설정할 수 있다.
				);

			const double _DERIVATIVE_STEP_SIZE = 1e-5; ///< 수치미분을 위한 step 크기입니다.

			unsigned int _NUM_VARIABLES; ///< 변수의 갯수를 저장하고 있는 변수
			unsigned int _NUM_CONSTRAINTS; ///< 조건의 갯수를 저장하고 있는 변수
		};
	}
}