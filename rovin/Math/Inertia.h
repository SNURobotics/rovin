/**
*	\file	Inertia.h
*	\date	2015.11.04
*	\author	Keunjun (ckj@robotics.snu.ac.kr)
*	\brief	Inertia 클래스를 정의
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
		*	\brief Inertia를 생성하고 처리하는 클래스
		*/
		class Inertia : public Eigen::Matrix<double, 6, 6>
		{
		public:
			/// 어떤 인자도 받지 않는 경우에는 0 매트릭스로 초기화를 합니다.
			Inertia() : Eigen::Matrix<double, 6, 6>(Eigen::Matrix<double, 6, 6>::Zero()) {}
			/// Ixx, Iyy, Izz, m을 차례로 받아 inertia 행렬을 구성합니다.
			Inertia(const double& Ixx, const double& Iyy, const double& Izz, const double& m);
			/// 질량 m만을 인자로 받는 경우는 Ixx = m. Iyy = m, Izz = m으로 생각하고 inertia 행렬을 구성합니다.
			Inertia(const double m) : Eigen::Matrix<double, 6, 6>(Eigen::Matrix<double, 6, 6>::Identity() * m) {}
			/// 회전관성 I와 질량 m을 받아 inertia 행렬을 구성합니다.
			Inertia(const Eigen::Matrix3d& I, const double& m);
			/// 회전관성 I, 질량중심의 위치, 질량 m을 입력받아 inertia 행렬을 구성합니다.
			Inertia(const Eigen::Matrix3d& I, const Eigen::Vector3d& p, const double& m);
			/// 회전관성 I = (Ixx, Iyy, Izz, Ixy, Ixz, Iyz), 질량중심의 위치 p = (p1, p2, p3), 질량 m을 입력받아 inertia 행렬을 구성합니다.
			Inertia(const Eigen::Matrix<double, 6, 1>& I, const Eigen::Vector3d& p, const double& m);
			/// srInertia과 같은 srInertia를 생성합니다.
			Inertia(const Inertia& I) : Eigen::Matrix<double, 6, 6>(I) {}
			/// Matrix 행렬을 받아서 srInertia를 생성합니다. 이때 행렬은 inertia 행렬의 성질을 만족해야 합니다.
			Inertia(const Eigen::Matrix<double, 6, 6>& I);

			/// srInertia 소멸자
			~Inertia() {}

			Inertia& operator = (const Inertia&);
			Inertia operator + (const Inertia&) const;
			Inertia& operator += (const Inertia&);
			Inertia operator * (const double) const;
			Inertia& operator *= (const double);
			Inertia operator / (const double) const;
			Inertia& operator /= (const double);

			/// 기준 frame을 T만큼 변환 시켜줍니다.
			void changeFrame(const SE3& T ///< 변환해주고 싶은 SE3
				);
		};
	}
}