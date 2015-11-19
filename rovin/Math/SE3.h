/**
*	\file	SE3.h
*	\date	2015.11.19
*	\author	Keunjun (ckj@robotics.snu.ac.kr)
*	\brief	SE3 클래스
*/

#pragma once

#include <list>

#include "Constant.h"

namespace rovin
{
	namespace Math
	{
		class SE3;
		class SE3List;
		typedef Vector6 se3;
		typedef Vector6 dse3;

		/**
		*	\class SE3
		*	\brief SE3를 생성하고 처리하는 클래스
		*/
		typedef Eigen::Matrix<double, 4, 4, Eigen::RowMajor> SE3Base;
		class SE3: public SE3Base
		{
		public:
			SE3() : SE3Base(SE3Base::Identity()) {}
			SE3(const SE3& T) : SE3Base(T) {}
			SE3(const double& T11, const double& T12, const double& T13, const double& T14,
				const double& T21, const double& T22, const double& T23, const double& T24,
				const double& T31, const double& T32, const double& T33, const double& T34)
			{
				double* T = &((*this)(0));
				(*T) = T11;
				(*(T + 1)) = T12;
				(*(T + 2)) = T13;
				(*(T + 3)) = T14;

				(*(T + 4)) = T21;
				(*(T + 5)) = T22;
				(*(T + 6)) = T23;
				(*(T + 7)) = T24;

				(*(T + 8)) = T31;
				(*(T + 9)) = T32;
				(*(T + 10)) = T33;
				(*(T + 11)) = T34;
			}

			SE3& operator = (const SE3& operand);
			SE3& operator = (const SE3List& operand);

			SE3& operator *= (const SE3& operand);
			SE3& operator *= (const SE3List& operand);

			//SE3List operator * (const SE3& operand) const;
			SE3 operator * (const SE3& operand) const;
			SE3List& operator * (SE3List& operand) const;

			template< typename Derived > friend
				const typename Eigen::ProductReturnType<Matrix4, Derived>::Type
				operator * (const SE3& operand1, const Eigen::MatrixBase< Derived >& operand2)
			{
				return (static_cast<Matrix4>(operand1)) * operand2;
			}

			template< typename Derived > friend
				const typename Eigen::ProductReturnType<Derived, Matrix4>::Type
				operator * (const Eigen::MatrixBase< Derived >& operand1, const SE3& operand2)
			{
				return operand2 * (static_cast<Matrix4>(operand1));
			}
		};

		class SE3List
		{
		public:
			friend SE3;

			void push_left(const SE3* item);
			void push_right(const SE3* item);

			operator SE3() const;
			operator Matrix4() const;

			SE3List& operator * (const SE3& operand);
			SE3List& operator * (SE3List& operand);
		
			template< typename Derived > friend
				const typename Eigen::ProductReturnType<Matrix4, Derived>::Type
				operator * (const SE3List& operand1, const Eigen::MatrixBase< Derived >& operand2)
			{
				return ((Matrix4)operand1) * operand2;
			}

			template< typename Derived > friend
				const typename Eigen::ProductReturnType<Derived, Matrix4>::Type
				operator * (const Eigen::MatrixBase< Derived >& operand1, const SE3List& operand2)
			{
				return operand1 * ((Matrix4)operand2);
			}

		private:
			static const unsigned int TempSize = 20;

			SE3List() : l(TempSize/2), r(TempSize/2) {}
			SE3List(const SE3* item1, const SE3* item2) : l(TempSize/2-1), r(TempSize/2+1)
			{
				Temp[TempSize / 2 - 1] = item1;
				Temp[TempSize / 2] = item2;
			}

			const SE3* Temp[TempSize];
			unsigned int l = TempSize /2;
			unsigned int r = TempSize /2;
		};
	}
}