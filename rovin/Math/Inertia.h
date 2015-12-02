/**
*	\file	Inertia.h
*	\date	2015.11.04
*	\author	Keunjun (ckj@robotics.snu.ac.kr)
*	\brief	Inertia Ŭ������ ����
*/

#pragma once

#include "Constant.h"
#include "LieGroup.h"

namespace rovin
{
	namespace Math
	{
		/**
		*	\class Inertia
		*	\brief Inertia�� �����ϰ� ó���ϴ� Ŭ����
		*/
		class Inertia : public Matrix6
		{
		public:
			/// �⺻������, Set Inertia to 6 by 6 identity matrix.
			Inertia() : Matrix6(Matrix6::Identity()) {}
			/// Ixx, Iyy, Izz, m�� ���ʷ� �޾� inertia ����� �����մϴ�.
			Inertia(const Real& Ixx, const Real& Iyy, const Real& Izz, const Real& m);
			/// ���� m���� ���ڷ� �޴� ���� Ixx = m. Iyy = m, Izz = m���� �����ϰ� inertia ����� �����մϴ�.
			Inertia(const Real m) : Matrix6(Matrix6::Identity() * m) {}
			/// ȸ������ I�� ���� m�� �޾� inertia ����� �����մϴ�.
			Inertia(const Matrix3& I, const Real& m);
			/// ȸ������ I, �����߽��� ��ġ, ���� m�� �Է¹޾� inertia ����� �����մϴ�.
			Inertia(const Matrix3& I, const Vector3& p, const Real& m);
			/// ȸ������ I = (Ixx, Iyy, Izz, Ixy, Ixz, Iyz), �����߽��� ��ġ p = (p1, p2, p3), ���� m�� �Է¹޾� inertia ����� �����մϴ�.
			Inertia(const Vector6& I, const Vector3& p, const Real& m);
			/// srInertia�� ���� srInertia�� �����մϴ�.
			Inertia(const Inertia& I) : Matrix6(I) {}
			/// Matrix ����� �޾Ƽ� srInertia�� �����մϴ�. �̶� ����� inertia ����� ������ �����ؾ� �մϴ�.
			Inertia(const Matrix6& I);

			/// srInertia �Ҹ���
			~Inertia() {}

			operator Matrix6();

			Inertia& operator = (const Inertia&);
			Inertia operator + (const Inertia&) const;
			Inertia& operator += (const Inertia&);
			Inertia operator * (const Real&) const;
			Inertia& operator *= (const Real&);
			Inertia operator / (const Real&) const;
			Inertia& operator /= (const Real&);

			/// ���� frame�� T��ŭ ��ȯ �����ݴϴ�.
			void changeFrame(const SE3& T ///< ��ȯ���ְ� ���� SE3
				);
		};
	}
}