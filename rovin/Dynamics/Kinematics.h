#pragma once

#include <string>

#include <rovin/Math/Constant.h>
#include <rovin/Math/LinearAlgebra.h>
#include <rovin/Model/Assembly.h>
#include <rovin/Model/SerialOpenChainAssembly.h>
#include <rovin/Model/State.h>

namespace rovin
{
	class Kinematics
	{
	public:
		enum FORWARD_KINEMATICS_OPTION
		{
			ACCUMULATED_T = 1 << 0,
			TRANSFORM = 1 << 1,
			ACCUMULATED_J = 1 << 2,
			VELOCITY = 1 << 3,
			ACCUMULATED_JDOT = 1 << 4,
			ACCELERATION = 1 << 5
		};

		static Math::VectorX computeClosedLoopConstraintFunction(const Model::Assembly& assem, Model::State& state);
		static Math::MatrixX computeClosedLoopConstraintJacobian(const Model::Assembly& assem, Model::State& state, const Model::State::TARGET_JOINT& return_state);
		static void solveClosedLoopConstraint(const Model::Assembly& assem, Model::State& state);

		//	Update position of each link which are corresponding to input state.
		static void solveForwardKinematics(const Model::Assembly& assem, Model::State& state, const unsigned int options = (TRANSFORM | VELOCITY | ACCELERATION));

		// Active Joint에 대해서 Jacobian을 구합니다. Closed Loop Constraint가 존재하는 경우에는 조건까지 포함하여 jacobian을 구해줍니다.
		static Math::Matrix6X computeJacobian(const Model::Assembly& assem, Model::State& state, const std::string& targetLinkName, const std::string& referenceLinkName = (""));
		static Math::Matrix6X computeJacobian(const Model::Assembly& assem, Model::State& state, unsigned int targetLinkIndex, int referenceLinkIndex = (-1));
		static std::pair< Math::SE3, Math::Matrix6X > computeTransformNJacobian(const Model::Assembly& assem, Model::State& state, unsigned int targetLinkIndex, int referenceLinkIndex = (-1));

		static void solveInverseKinematics(const Model::Assembly& assem, Model::State& state, const Math::SE3& goalT, const std::string& targetLinkName, const std::string& referenceLinkName = (""));
		static void solveInverseKinematics(const Model::Assembly& assem, Model::State& state, const Math::SE3& goalT, const unsigned int targetLinkIndex, int referenceLinkIndex = (-1));

		static void solveForwardKinematics(const Model::SerialOpenChainAssembly& assem, Model::State& state, const unsigned int options = (TRANSFORM | VELOCITY | ACCELERATION));
		static Math::SE3 calculateEndeffectorFrame(const Model::SerialOpenChainAssembly& assem, Model::State& state);

		static Math::Matrix6X computeJacobian(const Model::SerialOpenChainAssembly& assem, Model::State& state);
		static void solveInverseKinematics(const Model::SerialOpenChainAssembly& assem, Model::State& state, const Math::SE3 goalT);

	private:
		virtual ~Kinematics() = 0;
	};
}