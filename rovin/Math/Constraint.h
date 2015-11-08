/**
*	\file	Constraint.h
*	\date	2015.11.07
*	\author	Keunjun (ckj@robotics.snu.ac.kr)
*	\brief	Constraint�� ���õ� ������ �Լ��� ��� �ִ� Ŭ����
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
		*	\brief func(q)=0 �Ǵ� func(q)<=0 ������ q�� ���ؼ� Ǯ���ִ� Ŭ�����Դϴ�. �����Լ��Դϴ�.
		*/
		class Constraint
		{
		public:
			/// ������
			Constraint() : _NUM_VARIABLES(0), _NUM_CONSTRAINTS(0) {}
			/// ������ ������ Constraint�� ������ �����ݴϴ�.
			Constraint(const unsigned int& n_variables ///< ������ ����
				, const unsigned int& n_constraints ///< ������ ���� (��, func�� ���� ����)
				) : _NUM_VARIABLES(n_variables), _NUM_CONSTRAINTS(n_constraints) {}

			/// ������ ������ �����մϴ�.
			void setNumVariables(const unsigned int& n_variables ///< ������ ����
				)
			{
				_NUM_VARIABLES = n_variables;
			}
			/// ������ ������ �����մϴ�.
			void setNumConstraints(const unsigned int& n_constraints ///< ������ ����
				)
			{
				_NUM_CONSTRAINTS = n_constraints;
			}

			/// ������ ������ ������ �ɴϴ�.
			const unsigned int& getNumVariabales() const
			{
				return _NUM_VARIABLES;
			}
			/// ������ ������ ������ �ɴϴ�.
			const unsigned int& getNumConstraints() const
			{
				return _NUM_CONSTRAINTS;
			}

			/// Constraint�� �ش��ϴ� ���� �Լ��� ǥ���մϴ�. func(q)
			virtual Eigen::VectorXd func(const Eigen::VectorXd& q ///< ������ �޽��ϴ�.
				) = 0;
			virtual Eigen::MatrixXd Jacobian(const Eigen::VectorXd& q);

			/// Newton-Raphson method�� �̿��ؼ� func(q) = 0 ������ q�� ���ؼ� Ǭ��.
			Eigen::VectorXd sovleEq(const Eigen::VectorXd& initial_guess ///< �ʱⰪ�� ������ �� �ִ�.
				);

			const double _DERIVATIVE_STEP_SIZE = 1e-5; ///< ��ġ�̺��� ���� step ũ���Դϴ�.

			unsigned int _NUM_VARIABLES; ///< ������ ������ �����ϰ� �ִ� ����
			unsigned int _NUM_CONSTRAINTS; ///< ������ ������ �����ϰ� �ִ� ����
		};
	}
}