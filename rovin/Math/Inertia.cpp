/**
*	\file	Inertia.cpp
*	\date	2015.11.04
*	\author	Keunjun (ckj@robotics.snu.ac.kr)
*	\brief	Inertia 클래스 구현
*/

#include <cassert>

#include "Inertia.h"

namespace rovin
{
	namespace Math
	{
		Inertia::Inertia(const Real& Ixx, const Real& Iyy, const Real& Izz, const Real& m)
		{
			(*this) << (Matrix3)Eigen::DiagonalMatrix<Real, 3>(Ixx, Iyy, Izz), Matrix3::Zero(),
				Matrix3::Zero(), Matrix3::Identity()*m;
		}

		Inertia::Inertia(const Matrix3& I, const Real& m)
		{
			assert(I.isApprox(I.transpose()));
			(*this) << I, Matrix3::Zero(),
				Matrix3::Zero(), Matrix3::Identity()*m;
		}

		Inertia::Inertia(const Matrix3& I, const Vector3& p, const Real& m)
		{
			assert(I.isApprox(I.transpose()));
			(*this) << I, -Bracket(p),
				Bracket(p), Matrix3::Identity()*m;
		}

		Inertia::Inertia(const Vector6& I, const Vector3& p, const Real& m)
		{
			Matrix3 _I;
			_I << I(0), -I(3), -I(4),
				-I(3), I(1), -I(5),
				-I(4), -I(5), I(2);
			(*this) << _I, -Bracket(p),
				Bracket(p), Matrix3::Identity()*m;
		}

		Inertia::Inertia(const Matrix6& I)
		{
			Matrix3 _I = I.block(0, 0, 3, 3);
			Matrix3 _p = I.block(0, 3, 3, 3);
			Matrix3 _m = I.block(3, 3, 3, 3);

			assert(_I.isApprox(_I.transpose()));
			assert(abs(_p(0, 0)) < Eigen::NumTraits<Real>::dummy_precision() && abs(_p(1, 1)) < Eigen::NumTraits<Real>::dummy_precision() && abs(_p(2, 2)) < Eigen::NumTraits<Real>::dummy_precision());
			assert(_m.isApprox(Matrix3::Identity()*_m(0, 0)));

			(Matrix6)(*this) = I;
		}

		Inertia& Inertia::operator = (const Inertia& I)
		{
			(Matrix6)(*this) = (Matrix6&)I;
			return *this;
		}

		Inertia Inertia::operator + (const Inertia& I) const
		{
			Inertia result;
			(Matrix6)result = (Matrix6)(*this) + (Matrix6&)I;
			return result;
		}

		Inertia& Inertia::operator += (const Inertia& I)
		{
			(Matrix6)(*this) += (Matrix6&)I;
			return *this;
		}

		Inertia Inertia::operator * (const Real constant) const
		{
			Inertia result;
			(Matrix6)result = (Matrix6)(*this) * constant;
			return result;
		}

		Inertia& Inertia::operator *= (const Real constant)
		{
			(Matrix6)(*this) *= constant;
			return *this;
		}

		Inertia Inertia::operator / (const Real constant) const
		{
			Inertia result;
			(Matrix6)result = (Matrix6)(*this) / constant;
			return result;
		}

		Inertia& Inertia::operator /= (const Real constant)
		{
			(Matrix6)(*this) /= constant;
			return *this;
		}

		void Inertia::changeFrame(const SE3& T)
		{
			(Matrix6)(*this) = SE3::Ad(T).transpose() * (Matrix6)(*this) * SE3::Ad(T);
		}
	}
}