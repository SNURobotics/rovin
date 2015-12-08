/**
*	\file	SE3.h
*	\date	2015.11.05
*	\author	Keunjun (ckj@robotics.snu.ac.kr)
*	\brief	SE3 클래스
*/

#pragma once

#include <iostream>
#include "Constant.h"
#include "SO3.h"

namespace rovin
{
	namespace Math
	{
		class SE3;
		typedef Vector6 se3;
		typedef Vector6 dse3;

		/**
		*	\class SE3
		*	\brief SE3를 생성하고 처리하는 클래스
		*/
		class SE3
		{
		public:
			///생성자
			SE3(const SE3& T);
			/// 회전은 하지 않고 p만큼 이동한 SE3를 생성합니다. 
			SE3(const Vector3& p = (Vector3::Zero())) : _R(), _p(p) {}
			/// R만큼 회전을 하고 p만큼 이동한 SE3를 생성합니다.
			SE3(const SO3& R, const Vector3& p = (Vector3::Zero())) : _R(R), _p(p) {}
			/// R만큼 회전을 하고 p만큼 이동한 SE3를 생성합니다. 단, R이 임의의 3x3 행렬이므로 SO3를 만족하는지 확인을 합니다. SO3의 성질을 만족하지 않는 경우에는 에러가 납니다.
			SE3(const Matrix3& R, const Vector3& p = (Vector3::Zero())) : _R(R), _p(p) {}
			/// 임의의 행렬인 T를 받아서 T가 SE3를 만족하면 T를 이용하여 SE3를 생성합니다.
			SE3(const Matrix4& T);

			/// SE3 소멸자
			~SE3() {}

			SE3& operator = (const SE3&);
			SE3& operator = (const Matrix4&);
			SE3 operator * (const SE3&) const;
			SE3& operator *= (const SE3&);
			/**
			*	\warning SE3는 SE3끼리 계산하는 것이 빠르다.matrix로 변환하는 과정에서 시간이 많이 걸림.
			*	\brief Matrixr4d로 변환해줍니다.
			*/
			const Matrix4 matrix() const;

			/// 회전부분을 SO3인 R으로 설정합니다.
			void setRotation(const SO3& R);
			/// 회전부분을 임의의 행렬 M으로 설정합니다. 이 경우에는 _M이 SO3의 성질을 만족하는지 확인합니다.
			void setRotation(const Matrix3& M);
			/// Translation부분을 p로 설정합니다.
			void setPosition(const Vector3& p);
			/// 회전부분을 SO3형태로 알려줍니다.
			const SO3& getRotation() const;
			/// Translation부분을 Vector3d형태로 알려줍니다.
			const Vector3& getPosition() const;

			/// 출력 할 때 사용합니다.
			friend std::ostream& operator << (std::ostream&, const SE3&);

			/// 역행렬을 구해줍니다.
			SE3 inverse() const;

			/// se3인 S를 이용하여 exponential mapping을 해줍니다.
			static SE3 Exp(const se3& S, Real angle = (1.0));
			/// se3를 두 부분 (w, v)으로 입력을 받아 exponential mapping을 해줍니다.
			static SE3 Exp(so3 w, Vector3 v, Real angle = (1.0));

			/// Log 값을 계산해줍니다. 결과는 6x1 se3로 돌려줍니다.
			static se3 Log(const SE3&);

			/// T의 Large Adjoint를 해줍니다.
			static Matrix6 Ad(const SE3& T);
			///
			static se3 Ad(const SE3& T, const se3& S);
			/// T^(-1)의 Large Adjoint를 해줍니다.
			static Matrix6 InvAd(const SE3& T);
			///
			static se3 InvAd(const SE3& T, const se3& S);

			/// Small Adjoint인 [S]를 계산해줍니다.
			static Matrix6 ad(const se3& S);
			/// Small Adjoint인 [S]^T를 계산해줍니다.
			static Matrix6 adTranspose(const se3& S);
			///
			static se3 ad(const se3& S1, const se3& S2);
			static se3 adTranspose(const se3& S1, const se3& S2);

		private:
			SO3 _R;
			Vector3 _p;
		};

		/// [S]를 계산해줍니다.
		static Matrix4 Bracket(const se3& S)
		{
			Matrix4 result;

			result(0, 0) = 0;
			result(0, 1) = -S(2);
			result(0, 2) = S(1);

			result(1, 0) = S(2);
			result(1, 1) = 0;
			result(1, 2) = -S(0);

			result(2, 0) = -S(1);
			result(2, 1) = S(0);
			result(2, 2) = 0;

			result(0, 3) = S(3);
			result(1, 3) = S(4);
			result(2, 3) = S(5);

			result(3, 0) = 0;
			result(3, 1) = 0;
			result(3, 2) = 0;
			result(3, 3) = 0;

			return result;
		}
	}
}