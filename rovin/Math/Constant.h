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

		static const Real	PI_DOUBLE				= 6.28318530717958647692;
		static const Real	Inv_PI_DOUBLE			= 0.15915494309189533576901767873386;
		static const Real	PI						= 3.14159265358979323846;
		static const Real	PI_HALF					= 1.57079632679489661923;
		static const Real	PI_DIVIDED_BY_SQRT2		= 2.2214414690791831235079404950303;
		static const Real	PI_SQUARE				= 9.8696044010893586188344909998762;
		static const Real	DEG2RAD					= 0.01745329251994329577;
		static const Real	RAD2DEG					 = 57.2957795130823208768;

		static const Real	InverseKinematicsExitCondition = 1e-11;

		static const Real	RealEps = std::numeric_limits<Real>::epsilon();
		static const Real	RealMax = std::numeric_limits<Real>::max();
		static const Real	RealMin = std::numeric_limits<Real>::min();

		static bool RealEqual(const Real& operand1, const Real& operand2)
		{
			if (std::abs(operand1 - operand2) < RealEps + RealEps*std::abs(operand1))
			{
				return true;
			}
			return false;
		}

		static bool RealEqual(const VectorX& operand1, const Real& operand2)
		{
			unsigned int n = operand1.size();
			for (unsigned int i = 0; i < n; i++)
			{
				if (!RealEqual(operand1(i), operand2))
				{
					return false;
				}
			}
			return true;
		}

		static bool RealLessEqual(const Real& operand1, const Real& operand2)
		{
			if (operand1 < operand2 + RealEps + RealEps*std::abs(operand1))
			{
				return true;
			}
			return false;
		}

		static bool RealLessEqual(const VectorX& operand1, const Real& operand2)
		{
			unsigned int n = operand1.size();
			for (unsigned int i = 0; i < n; i++)
			{
				if (!RealLessEqual(operand1(i), operand2))
				{
					return false;
				}
			}
			return true;
		}

		static bool RealLess(const Real& operand1, const Real& operand2)
		{
			if (operand1 < operand2 - RealEps - RealEps*std::abs(operand1))
			{
				return true;
			}
			return false;
		}

		static bool RealBiggerEqual(const Real& operand1, const Real& operand2)
		{
			if (operand1 > operand2 - RealEps - RealEps*std::abs(operand1))
			{
				return true;
			}
			return false;
		}

		static bool RealBigger(const Real& operand1, const Real& operand2)
		{
			if (operand1 > operand2 + RealEps + RealEps*std::abs(operand1))
			{
				return true;
			}
			return false;
		}
	}
}