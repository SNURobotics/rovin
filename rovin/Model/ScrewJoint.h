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
			void	adjointAxes(const Math::SE3&	TransformFromJoint);


			virtual JointPtr copy() const;

			virtual Math::SE3	getTransform(const Math::VectorX& state, bool isReversed = false) const override;
			virtual Math::se3	getVelocity(const Math::VectorX& state, bool isReversed = false) const override;
			virtual Math::Matrix6X getJacobian(const Math::VectorX& state, bool isReversed = false) const override;
			virtual Math::Matrix6X getJacobianDot(const Math::VectorX& state) const override;
			
			virtual void	updateForwardKinematics(State::JointState& state, JointDirection direction = REGULAR, bool position = true, bool velocity = false, bool acceleration = false) const override;
			virtual Math::Matrix6X getJacobian(State::JointState& state, JointDirection direction = REGULAR, bool updateTransform = false) const override;

		protected:
			///	Vector of Math::se3 which contatin rotation axis of each joint
			Math::Matrix6X		_axes;
		};
	}
}
