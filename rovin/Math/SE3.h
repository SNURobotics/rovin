/**
*	\file	SE3.h
*	\date	2015.11.05
*	\author	Keunjun (ckj@robotics.snu.ac.kr)
*	\brief	SE3 클래스
*/

#pragma once

#include <iostream>
#include <Eigen/Dense>
#include "SO3.h"

namespace rovin
{
	namespace Math
	{
		class SE3;
		typedef Eigen::Matrix<double, 6, 1> se3;
		typedef Eigen::Matrix<double, 6, 1> dse3;

		/**
		*	\class SE3
		*	\brief SE3를 생성하고 처리하는 클래스
		*/
		class SE3
		{
		public:
			/// 회전은 하지 않고 p만큼 이동한 SE3를 생성합니다. 
			SE3(const Eigen::Vector3d& p = (Eigen::Vector3d::Zero())) : _R(), _p(p) {}
			/// R만큼 회전을 하고 p만큼 이동한 SE3를 생성합니다.
			SE3(const SO3& R, const Eigen::Vector3d& p = (Eigen::Vector3d::Zero())) : _R(R), _p(p) {}
			/// R만큼 회전을 하고 p만큼 이동한 SE3를 생성합니다. 단, R이 임의의 3x3 행렬이므로 SO3를 만족하는지 확인을 합니다. SO3의 성질을 만족하지 않는 경우에는 에러가 납니다.
			SE3(const Eigen::Matrix3d& R, const Eigen::Vector3d& p = (Eigen::Vector3d::Zero())) : _R(R), _p(p) {}
			/// 임의의 행렬인 T를 받아서 T가 SE3를 만족하면 T를 이용하여 SE3를 생성합니다.
			SE3(const Eigen::Matrix4d& T);

			/// SE3 소멸자
			~SE3() {}

			SE3& operator = (const SE3&);
			SE3& operator = (const Eigen::Matrix4d&);
			SE3 operator * (const SE3&) const;
			SE3& operator *= (const SE3&);
			/**
			*	\warning SE3는 SE3끼리 계산하는 것이 빠르다.matrix로 변환하는 과정에서 시간이 많이 걸림.
			*	\brief Matrixr4d로 변환해줍니다.
			*/
			const Eigen::Matrix4d matrix() const;

			/// 회전부분을 SO3인 R으로 설정합니다.
			void setRotation(const SO3& R);
			/// 회전부분을 임의의 행렬 M으로 설정합니다. 이 경우에는 _M이 SO3의 성질을 만족하는지 확인합니다.
			void setRotation(const Eigen::Matrix3d& M);
			/// Translation부분을 p로 설정합니다.
			void setPosition(const Eigen::Vector3d& p);
			/// 회전부분을 SO3형태로 알려줍니다.
			const SO3& getRotation() const;
			/// Translation부분을 Vector3d형태로 알려줍니다.
			const Eigen::Vector3d& getPosition() const;

			/// 출력 할 때 사용합니다.
			friend std::ostream& operator << (std::ostream&, const SE3&);

			/// 역행렬을 구해줍니다.
			SE3 inverse() const;

			/// se3인 S를 이용하여 exponential mapping을 해줍니다.
			static SE3 Exp(const se3& S, const double angle = (1.0));
			/// se3를 두 부분 (w, v)으로 입력을 받아 exponential mapping을 해줍니다.
			static SE3 Exp(const so3& w, const Eigen::Vector3d& v, const double angle = (1.0));

			/// Log 값을 계산해줍니다. 결과는 6x1 se3로 돌려줍니다.
			static se3 Log(const SE3&);

			/// T의 Large Adjoint를 해줍니다.
			static Eigen::Matrix<double, 6, 6> Ad(const SE3& T);
			/// T^(-1)의 Large Adjoint를 해줍니다.
			static Eigen::Matrix<double, 6, 6> invAd(const SE3& T);

			/// Small Adjoint인 [S]를 계산해줍니다.
			static Eigen::Matrix<double, 6, 6> ad(const se3& S);

		private:
			SO3 _R;
			Eigen::Vector3d _p;
		};

		/// [S]를 계산해줍니다.
		static Eigen::Matrix4d Bracket(const se3& S)
		{
			so3 w = S.block(0, 0, 3, 1);
			Eigen::Vector3d v = S.block(3, 0, 3, 1);

			Eigen::Matrix4d result;
			result.setZero();
			result.block(0, 0, 3, 3) = Bracket(w);
			result.block(0, 3, 3, 1) = v;
			return result;
		}
	}
}