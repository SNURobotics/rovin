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
			ScrewJoint(const std::string& name = std::string(""), unsigned int dof = 1, const Math::MatrixX& axes = Math::MatrixX());
			ScrewJoint(const ScrewJoint& otherJoint);
			virtual ~ScrewJoint() = default;

			const Math::MatrixX&	getAxes() const { return _axes; }

			void					setAxis(const Math::Vector3& axis, unsigned int index);
			void					setAxis(const Math::Vector6& axis, unsigned int index);


			ScrewJoint& operator=(const ScrewJoint& otherJoint);

			virtual std::shared_ptr<Joint> copy() const;

			virtual Math::SE3	getTransform(const Math::VectorX& state) const override;
			virtual Math::MatrixX getJacobian(const Math::VectorX& state) const override;
			virtual Math::MatrixX getJacobianTimeDeriv(const Math::VectorX& state) const override;


		protected:
			///	3 by DOF matrix. i-th column contatin rotation axis of i-th joint
			Math::MatrixX		_axes;
		};
	}
}
