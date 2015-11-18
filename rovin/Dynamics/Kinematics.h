#pragma once

#include <rovin/Math/Constant.h>
#include <rovin/Math/LinearAlgebra.h>
#include <rovin/Model/Assembly.h>
#include <rovin/Model/State.h>

namespace rovin
{
	class Kinematics
	{
	public:
		static Math::VectorX computeClosedLoopConstraintFunction(const Model::Assembly& assem, const Model::State& state);
		static Math::MatrixX computeClosedLoopConstraintJacobian(const Model::Assembly& assem, const Model::State& state, const Model::State::RETURN_STATE& return_state);
		static void solveClosedLoopConstraint(const Model::Assembly& assem, Model::State& state);

		//	Update position and velocity of each link which are corresponding to input state.
		static void solveForwardKinematics(const Model::Assembly& assem, Model::State& state);
	private:
		virtual ~Kinematics() = 0;
	};
}