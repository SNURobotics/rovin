#pragma once

#include <string>

#include <rovin/Math/Constant.h>
#include <rovin/Math/LinearAlgebra.h>
#include <rovin/Model/Assembly.h>
#include <rovin/Model/SerialOpenChainAssembly.h>
#include <rovin/Model/State.h>
#include <rovin/Dynamics/Kinematics.h>

namespace rovin
{
	class Dynamics
	{
	public:
		static void solveInverseDynamics(const Model::SerialOpenChainAssembly& assem, Model::State& state, const std::vector< std::pair< std::string, Math::dse3 >>& F = (std::vector< std::pair< std::string, Math::dse3 >>()));
		static void solveInverseDynamics(const Model::SerialOpenChainAssembly& assem, Model::State& state, const std::vector< std::pair< unsigned int, Math::dse3 >>& F = (std::vector< std::pair< unsigned int, Math::dse3 >>()));

	private:
		virtual ~Dynamics() = 0;
	};
}