/**
*	\file	Revolutejoint.h
*	\date	2015.11.09
*	\author	Jisoo Hong(jshong@robotics.snu.ac.kr)
*	\brief	Specify kinematic information of general screw joint.
*/

#pragma once

#include <string>
#include <memory>
#include <rovin/Math/LieGroup.h>
#include "Joint.h"

namespace rovin {
	namespace Model {
		class ScrewJoint : public Joint
		{
		public:
			ScrewJoint(const std::string& name = std::string(""), unsigned int dof = 1, const Math::MatrixX& axes = Math::Matrix6X());
			ScrewJoint(const ScrewJoint& otherJoint);
			virtual ~ScrewJoint() = default;
			ScrewJoint& operator=(const ScrewJoint& otherJoint);

			const Math::Matrix6X&	getAxes() const { return _axes; }

			void					setAxis(const Math::Vector3& axis, unsigned int index);
			void					setAxis(const Math::Vector6& axis, unsigned int index);

			void	normalizeAxes();
			//void	adjointAxes(const Math::SE3&	TransformFromJoint);


			virtual JointPtr copy() const;

			virtual void updateTransform(State::JointState& state) const override;
			virtual void updateJacobian(State::JointState& state) const override;
			virtual void updateJacobianDot(State::JointState& state) const override;

		protected:
			///	Vector of Math::se3 which contatin rotation axis of each joint
			Math::Matrix6X		_axes;
		};
	}
}
