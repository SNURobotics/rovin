/**
*	\file	Revolutejoint.h
*	\date	2015.11.11
*	\author	Jisoo Hong(jshong@robotics.snu.ac.kr)
*	\brief	Specify kinematic information of revolute joint.
*/

#pragma once

#include "ScrewJoint.h"


namespace rovin {
	namespace Model {
		class RevoluteJoint : public ScrewJoint
		{
		public:
			RevoluteJoint(const std::string& name = std::string(""), const Math::Vector3& axis = Math::Vector3::UnitZ())
				:ScrewJoint(name, 1, axis)
			{

				// do nothing
			}
			RevoluteJoint(const RevoluteJoint& otherJoint)
				:ScrewJoint(otherJoint) {}
			virtual ~RevoluteJoint() = default;

			RevoluteJoint& operator=(const RevoluteJoint& otherJoint)
			{
				ScrewJoint::operator=(otherJoint);
				return *this;
			}


		};
	}
}