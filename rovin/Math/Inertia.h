/**
*	\file	Inertia.h
*	\date	2015.11.04
*	\author	Keunjun (ckj@robotics.snu.ac.kr)
*	\brief	Inertia Ŭ������ ����
*/

#pragma once

#include <Eigen/Dense>
#include "LieGroup.h"

namespace rovin
{
	namespace Math
	{
		/**
		*	\class Inertia
		*	\brief Inertia�� �����ϰ� ó���ϴ� Ŭ����
		*/
		class Inertia : public Eigen::Matrix<double, 6, 6>
		{
		public:
			/// � ���ڵ� ���� �ʴ� ��쿡�� 0 ��Ʈ������ �ʱ�ȭ�� �մϴ�.
			Inertia() : Eigen::Matrix<double, 6, 6>(Eigen::Matrix<double, 6, 6>::Zero()) {}
			/// Ixx, Iyy, Izz, m�� ���ʷ� �޾� inertia ����� �����մϴ�.
			Inertia(const double& Ixx, const double& Iyy, const double& Izz, const double& m);
			/// ���� m���� ���ڷ� �޴� ���� Ixx = m. Iyy = m, Izz = m���� �����ϰ� inertia ����� �����մϴ�.
			Inertia(const double m) : Eigen::Matrix<double, 6, 6>(Eigen::Matrix<double, 6, 6>::Identity() * m) {}
			/// ȸ������ I�� ���� m�� �޾� inertia ����� �����մϴ�.
			Inertia(const Eigen::Matrix3d& I, const double& m);
			/// ȸ������ I, �����߽��� ��ġ, ���� m�� �Է¹޾� inertia ����� �����մϴ�.
			Inertia(const Eigen::Matrix3d& I, const Eigen::Vector3d& p, const double& m);
			/// ȸ������ I = (Ixx, Iyy, Izz, Ixy, Ixz, Iyz), �����߽��� ��ġ p = (p1, p2, p3), ���� m�� �Է¹޾� inertia ����� �����մϴ�.
			Inertia(const Eigen::Matrix<double, 6, 1>& I, const Eigen::Vector3d& p, const double& m);
			/// srInertia�� ���� srInertia�� �����մϴ�.
			Inertia(const Inertia& I) : Eigen::Matrix<double, 6, 6>(I) {}
			/// Matrix ����� �޾Ƽ� srInertia�� �����մϴ�. �̶� ����� inertia ����� ������ �����ؾ� �մϴ�.
			Inertia(const Eigen::Matrix<double, 6, 6>& I);

			/// srInertia �Ҹ���
			~Inertia() {}

			Inertia& operator = (const Inertia&);
			Inertia operator + (const Inertia&) const;
			Inertia& operator += (const Inertia&);
			Inertia operator * (const double) const;
			Inertia& operator *= (const double);
			Inertia operator / (const double) const;
			Inertia& operator /= (const double);

			/// ���� frame�� T��ŭ ��ȯ �����ݴϴ�.
			void changeFrame(const SE3& T ///< ��ȯ���ְ� ���� SE3
				);
		};
	}
}