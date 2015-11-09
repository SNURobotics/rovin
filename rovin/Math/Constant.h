/**
*	\file	Constant.h
*	\date	2015.11.09
*	\author	Jisoo Hong(jshong@robotics.snu.ac.kr)
*	\brief	Define type and useful constant.
*/

#pragma once

#include <Eigen\Dense>

namespace rovin
{
	namespace Math {
		//typedef float	real;
		typedef	double	real;

		typedef Eigen::Matrix<real, -1, -1>		MatrixX;
		typedef Eigen::Matrix<real, 4, 4>		Matrix4;
		typedef Eigen::Matrix<real, -1, 1>		VectorX;
		typedef Eigen::Matrix<real, 3, 1>		Vector3;
		typedef Eigen::Matrix<real, 6, 1>		Vector6;

		static const real	PI = 3.14159265358979323846;
		static const real	PI_HALF = 1.57079632679489661923;
		static const real	DEG2RAD = 0.01745329251994329577;
		static const real	RAD2DEG = 57.2957795130823208768;
	}
}