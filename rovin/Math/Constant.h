/**
*	\file	Constant.h
*	\date	2015.11.09
*	\author	Jisoo Hong(jshong@robotics.snu.ac.kr)
*	\brief	Define type and useful constant.
*/

#pragma once

#include <Eigen/Dense>

namespace rovin
{
	namespace Math {
		//typedef float	Real;
		typedef	double	Real;

		typedef Eigen::Matrix<Real, -1, -1>		MatrixX;
		typedef Eigen::Matrix<Real, 3, 3>		Matrix3;
		typedef Eigen::Matrix<Real, 4, 4>		Matrix4;
		typedef Eigen::Matrix<Real, 6, 6>		Matrix6;
		typedef Eigen::Matrix<Real, 6, -1>		Matrix6X;
		typedef Eigen::Matrix<Real, -1, 1>		VectorX;
		typedef Eigen::Matrix<Real, 2, 1>		Vector2;
		typedef Eigen::Matrix<Real, 3, 1>		Vector3;
		typedef Eigen::Matrix<Real, 4, 1>		Vector4;
		typedef Eigen::Matrix<Real, 6, 1>		Vector6;

		static const Real	PI = 3.14159265358979323846;
		static const Real	PI_HALF = 1.57079632679489661923;
		static const Real	DEG2RAD = 0.01745329251994329577;
		static const Real	RAD2DEG = 57.2957795130823208768;

		static const Real	RealEps = std::numeric_limits<Real>::epsilon();
		static const Real	RealMax = std::numeric_limits<Real>::max();
		static const Real	RealMin = std::numeric_limits<Real>::min();
	}
}