/**
*	\file	PrismaticJoint.h
*	\date	2015.11.11
*	\author	Jisoo Hong(jshong@robotics.snu.ac.kr)
*	\brief	Specify kinematic information of prismatic joint.
*/

#pragma once


#include "ScrewJoint.h"

namespace rovin {
	namespace Model {
		class PrismaticJoint : public ScrewJoint
		{
		public:
			PrismaticJoint(const std::string& name = std::string(""), const Math::Vector3& axis = Math::Vector3::UnitZ())
				:ScrewJoint::ScrewJoint(name, 1)
			{
				Math::Vector6	axis6 = Math::Vector6::Zero();
				axis6.tail<3>() = axis;
				_axes = axis6;
			}
			PrismaticJoint(const PrismaticJoint& otherJoint)
				:ScrewJoint(otherJoint) {}
			virtual ~PrismaticJoint() = default;

			PrismaticJoint& operator=(const PrismaticJoint& otherJoint)
			{
				ScrewJoint::operator=(otherJoint);
				return *this;
			}
		};
	}
}