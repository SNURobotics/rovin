#pragma once

#include "Assembly.h"

namespace rovin
{
	class Kinematics;
	class Dynamics;

	namespace Model
	{
		class SerialOpenChainAssembly;

		typedef SerialOpenChainAssembly socAssembly;
		typedef std::shared_ptr< SerialOpenChainAssembly > socAssemblyPtr;

		class SerialOpenChainAssembly : public Assembly
		{
			friend class Kinematics;
			friend class Dynamics;

		public:
			class SerialOpenChainLink
			{
			public:
				Math::SE3		_M;
				Math::Inertia	_G;

			public:
				EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			};

			class SerialOpenChainMate
			{
			public:
				Math::Matrix6X _axes;

			public:
				EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			};

			SerialOpenChainAssembly(const std::string& name) : Assembly(name) {}

			StatePtr makeState() const;

			void updateJointKinematics(const unsigned int mateIdx, State::JointState& jointState, const unsigned int options) const;

			const Math::SE3& getTransform(const unsigned int mateIdx, State::JointState& jointState) const;
			const Math::Matrix6X& getJacobian(const unsigned int mateIdx, State::JointState& jointState) const;
			const Math::Matrix6X& getJacobianDot(const unsigned int mateIdx, State::JointState& jointState) const;

			void completeAssembling(const std::string& baseLinkName);

		private:
			unsigned int _endeffectorLink;

			std::vector< SerialOpenChainLink, Eigen::aligned_allocator< SerialOpenChainLink >> _socLink;
			std::vector< SerialOpenChainMate, Eigen::aligned_allocator< SerialOpenChainMate >> _socMate;
		};
	}
}