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
				Math::Vector6 screw = Math::Vector6::Zero();
				screw.tail<3>() = axis;
				setAxis(screw, 0);
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