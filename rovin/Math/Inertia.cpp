/**
*	\file	Inertia.cpp
*	\date	2015.11.04
*	\author	Keunjun (ckj@robotics.snu.ac.kr)
*	\brief	Inertia 클래스 구현
*/

#include <cassert>

#include "Inertia.h"

using namespace Eigen;

namespace rovin
{
	namespace Math
	{
		Inertia::Inertia(const double& Ixx, const double& Iyy, const double& Izz, const double& m)
		{
			(*this) << (Matrix3d)DiagonalMatrix<double, 3>(Ixx, Iyy, Izz), Matrix3d::Zero(),
				Matrix3d::Zero(), Matrix3d::Identity()*m;
		}

		Inertia::Inertia(const Matrix3d& I, const double& m)
		{
			assert(I.isApprox(I.transpose().eval()));
			(*this) << I, Matrix3d::Zero(),
				Matrix3d::Zero(), Matrix3d::Identity()*m;
		}

		Inertia::Inertia(const Matrix3d& I, const Vector3d& p, const double& m)
		{
			assert(I.isApprox(I.transpose().eval()));
			(*this) << I, -Bracket(p),
				Bracket(p), Matrix3d::Identity()*m;
		}

		Inertia::Inertia(const Matrix<double, 6, 1>& I, const Vector3d& p, const double& m)
		{
			Matrix3d _I;
			_I << I(0), -I(3), -I(4),
				-I(3), I(1), -I(5),
				-I(4), -I(5), I(2);
			(*this) << _I, -Bracket(p),
				Bracket(p), Matrix3d::Identity()*m;
		}

		Inertia::Inertia(const Matrix<double, 6, 6>& I)
		{
			Matrix3d _I = I.block(0, 0, 3, 3);
			Matrix3d _p = I.block(0, 3, 3, 3);
			Matrix3d _m = I.block(3, 3, 3, 3);

			assert(_I.isApprox(_I.transpose()));
			assert(abs(_p(0, 0)) < NumTraits<double>::dummy_precision() && abs(_p(1, 1)) < NumTraits<double>::dummy_precision() && abs(_p(2, 2)) < NumTraits<double>::dummy_precision());
			assert(_m.isApprox(Matrix3d::Identity()*_m(0, 0)));

			(Matrix<double, 6, 6>)(*this) = I;
		}

		Inertia& Inertia::operator = (const Inertia& I)
		{
			(Matrix<double, 6, 6>)(*this) = (Matrix<double, 6, 6>&)I;
			return *this;
		}

		Inertia Inertia::operator + (const Inertia& I) const
		{
			Inertia result;
			(Matrix<double, 6, 6>)result = (Matrix<double, 6, 6>)(*this) + (Matrix<double, 6, 6>&)I;
			return result;
		}

		Inertia& Inertia::operator += (const Inertia& I)
		{
			(Matrix<double, 6, 6>)(*this) += (Matrix<double, 6, 6>&)I;
			return *this;
		}

		Inertia Inertia::operator * (const double constant) const
		{
			Inertia result;
			(Matrix<double, 6, 6>)result = (Matrix<double, 6, 6>)(*this) * constant;
			return result;
		}

		Inertia& Inertia::operator *= (const double constant)
		{
			(Matrix<double, 6, 6>)(*this) *= constant;
			return *this;
		}

		Inertia Inertia::operator / (const double constant) const
		{
			Inertia result;
			(Matrix<double, 6, 6>)result = (Matrix<double, 6, 6>)(*this) / constant;
			return result;
		}

		Inertia& Inertia::operator /= (const double constant)
		{
			(Matrix<double, 6, 6>)(*this) /= constant;
			return *this;
		}

		void Inertia::changeFrame(const SE3& T)
		{
			(Matrix<double, 6, 6>)(*this) = SE3::Ad(T).transpose() * (Matrix<double, 6, 6>)(*this) * SE3::Ad(T);
		}
	}
}