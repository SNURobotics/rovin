#pragma once

#include <string>

#include <rovin/Math/Constant.h>
#include <rovin/Math/Common.h>
#include <rovin/Model/Assembly.h>
#include <rovin/Model/SerialOpenChainAssembly.h>
#include <rovin/Model/State.h>
#include <rovin/Dynamics/Kinematics.h>

namespace rovin
{
	class Dynamics
	{
	public:
		static void solveInverseDynamics(const Model::SerialOpenChainAssembly& assem, Model::State& state);
		static void solveInverseDynamics(const Model::SerialOpenChainAssembly& assem, Model::State& state, const std::vector< std::pair< std::string, Math::dse3 >>& F );
		static void solveInverseDynamics(const Model::SerialOpenChainAssembly& assem, Model::State& state, const std::vector< std::pair< unsigned int, Math::dse3 >>& F);
		

		// differentiateInverseDynamics should be called after calling forward kinematics & inverse dynamics (dFextdp, d2Fextdp2 should be calculated from current position)
		static std::pair<Math::MatrixX, std::vector<Math::MatrixX>> differentiateInverseDynamics(const Model::SerialOpenChainAssembly& assem, Model::State& state,
			const Math::MatrixX& dqdp, const Math::MatrixX& dqdotdp, const Math::MatrixX& dqddotdp, 
			const std::vector<Math::MatrixX>& d2qdp2 = (std::vector<Math::MatrixX>()),
			const std::vector<Math::MatrixX>& d2qdotdp2 = (std::vector<Math::MatrixX>()),
			const std::vector<Math::MatrixX>& d2qddotdp2 = (std::vector<Math::MatrixX>()),
			const std::vector<std::pair< unsigned int, Math::dse3>>& F = (std::vector<std::pair< unsigned int, Math::dse3>>()), 
			const std::vector<std::pair<unsigned int, Math::MatrixX >>& dFextdp = (std::vector<std::pair<unsigned int, Math::MatrixX >>()), 
			const std::vector<std::pair<unsigned int, std::vector< Math::MatrixX >>>& d2Fextdp2 = (std::vector<std::pair<unsigned int, std::vector< Math::MatrixX >>>()),
			const bool needInverseDynamics = (false));

		static void solveForwardDynamics(const Model::SerialOpenChainAssembly& assem, Model::State& state, const std::vector< std::pair< unsigned int, Math::dse3 >>& extForce = (std::vector< std::pair< unsigned int, Math::dse3 >>()));
	private:
		virtual ~Dynamics() = 0;
	};
}