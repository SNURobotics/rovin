/**
*	\file	Kinematics.h
*	\date	2015.11.09
*	\author	Keunjun (ckj@robotics.snu.ac.kr)
*	\brief	Kinematics관련 함수 모음
*/

#pragma once

#include <rovin/Math/Constant.h>

#include "System.h"
#include "State.h"

namespace rovin
{
	namespace Dynamics
	{
		static Math::VectorX ClosedLoop_Constraint_Function(const System& sys, const State& state)
		{

		}
		static Math::MatrixX ClosedLoop_Constraint_Jacobian(const System& sys, const State& state)
		{

		}
		static Math::MatrixX ClosedLoop_Constraint_Jacobian_dot(const System& sys, const State& state)
		{

		}
	}
}