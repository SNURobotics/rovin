/**
*	\file	Inertia.h
*	\date	2015.11.04
*	\author	Keunjun (ckj@robotics.snu.ac.kr)
*	\brief	Inertia 클래스를 정의
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
		*	\brief Inertia를 생성하고 처리하는 클래스
		*/
		class Inertia : public Matrix6
		{
		public:
			/// 기본생성자, Set Inertia to 6 by 6 identity matrix.
			Inertia() : Matrix6(Matrix6::Identity()) {}
			/// Ixx, Iyy, Izz, m을 차례로 받아 inertia 행렬을 구성합니다.
			Inertia(const Real& Ixx, const Real& Iyy, const Real& Izz, const Real& m);
			/// 질량 m만을 인자로 받는 경우는 Ixx = m. Iyy = m, Izz = m으로 생각하고 inertia 행렬을 구성합니다.
			Inertia(const Real m) : Matrix6(Matrix6::Identity() * m) {}
			/// 회전관성 I와 질량 m을 받아 inertia 행렬을 구성합니다.
			Inertia(const Matrix3& I, const Real& m);
			/// 회전관성 I, 질량중심의 위치, 질량 m을 입력받아 inertia 행렬을 구성합니다.
			Inertia(const Matrix3& I, const Vector3& p, const Real& m);
			/// 회전관성 I = (Ixx, Iyy, Izz, Ixy, Ixz, Iyz), 질량중심의 위치 p = (p1, p2, p3), 질량 m을 입력받아 inertia 행렬을 구성합니다.
			Inertia(const Vector6& I, const Vector3& p, const Real& m);
			/// srInertia과 같은 srInertia를 생성합니다.
			Inertia(const Inertia& I) : Matrix6(I) {}
			/// Matrix 행렬을 받아서 srInertia를 생성합니다. 이때 행렬은 inertia 행렬의 성질을 만족해야 합니다.
			Inertia(const Matrix6& I);

			/// srInertia 소멸자
			~Inertia() {}

			operator Matrix6();

			Inertia& operator = (const Inertia&);
			Inertia operator + (const Inertia&) const;
			Inertia& operator += (const Inertia&);
			Inertia operator * (const Real&) const;
			Inertia& operator *= (const Real&);
			Inertia operator / (const Real&) const;
			Inertia& operator /= (const Real&);

			/// 기준 frame을 T만큼 변환 시켜줍니다.
			void changeFrame(const SE3& T ///< 변환해주고 싶은 SE3
				);
		};
	}
}