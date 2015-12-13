#pragma once

#include <string>
#include <vector>

#include <rovin/Math/Constant.h>
#include <rovin/Math/Common.h>
#include <rovin/Model/Assembly.h>
#include <rovin/Model/SerialOpenChainAssembly.h>
#include <rovin/Model/State.h>

namespace rovin
{
	using namespace Model;
	class Kinematics
	{
	public:
		static Math::VectorX computeClosedLoopConstraintFunction(const Model::Assembly& assem, Model::State& state);
		static Math::MatrixX computeClosedLoopConstraintJacobian(const Model::Assembly& assem, Model::State& state, const Model::State::TARGET_JOINT& return_state);
		static void solveClosedLoopConstraint(const Model::Assembly& assem, Model::State& state);

		//	Update position of each link which are corresponding to input state.
		static void solveForwardKinematics(const Model::Assembly& assem, Model::State& state, const unsigned int options = (State::LINKS_POS | State::LINKS_VEL | State::LINKS_ACC));

		// Active Joint�� ���ؼ� Jacobian�� ���մϴ�. Closed Loop Constraint�� �����ϴ� ��쿡�� ���Ǳ��� �����Ͽ� jacobian�� �����ݴϴ�.
		static Math::Matrix6X computeJacobian(const Model::Assembly& assem, Model::State& state, const std::string& targetLinkName, const std::string& referenceLinkName = (""));
		static Math::Matrix6X computeJacobian(const Model::Assembly& assem, Model::State& state, unsigned int targetLinkIndex, int referenceLinkIndex = (-1));
		static std::pair< Math::SE3, Math::Matrix6X > computeTransformNJacobian(const Model::Assembly& assem, Model::State& state, unsigned int targetLinkIndex, int referenceLinkIndex = (-1));

		static void solveInverseKinematics(const Model::Assembly& assem, Model::State& state, const Math::SE3& goalT, const std::string& targetLinkName, const std::string& referenceLinkName = (""));
		static void solveInverseKinematics(const Model::Assembly& assem, Model::State& state, const Math::SE3& goalT, const unsigned int targetLinkIndex, int referenceLinkIndex = (-1));

		static void solveForwardKinematics(const Model::SerialOpenChainAssembly& assem, Model::State& state, const unsigned int options = (State::LINKS_POS | State::LINKS_VEL | State::LINKS_ACC));
		static Math::SE3 calculateEndeffectorFrame(const Model::SerialOpenChainAssembly& assem, Model::State& state);

		static Math::Matrix6X computeJacobian(const Model::SerialOpenChainAssembly& assem, Model::State& state);
		static Math::Matrix6X computeJacobianDot(const Model::SerialOpenChainAssembly& assem, Model::State& state);
		static void solveInverseKinematics(const Model::SerialOpenChainAssembly& assem, Model::State& state, const Math::SE3 goalT);
		static void solveInverseKinematics(const Model::SerialOpenChainAssembly& assem, Model::State& state, const Math::Vector3 goalPosition, const Math::SE3& tip = (Math::SE3()));

		static std::vector<Math::VectorX> solveInverseKinematicsOnlyForEfort(const Model::SerialOpenChainAssembly& assem, const Math::SE3& goalT);

	private:
		virtual ~Kinematics() = 0;
	};
}