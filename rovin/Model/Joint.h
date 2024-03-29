/**
*	\file	Joint.h
*	\date	2015.11.06
*	\author	Jisoo Hong(jshong@robotics.snu.ac.kr), Keunjun (ckj@robotics.snu.ac.kr)
*	\brief	Joint의 정보를 저장하는 클래스 정의
*/

#pragma once

#include <string>
#include <memory>

#include <Eigen/Dense>
#include <rovin/Math/LieGroup.h>
#include <rovin/Math/Constant.h>
#include <rovin/Model/State.h>

namespace rovin
{
	namespace Model
	{
		class Link;
		class Joint;
		class JointState;

		typedef std::shared_ptr< Joint > JointPtr;

		/**
		*	\class Joint
		*	\brief Joint class specify kinematic information about general mechanical joints.
		*/
		class Joint
		{
			friend class Assembly;

		public:
			enum JOINTTYPE { NONE, SCREWJOINT };

			Joint(const JOINTTYPE& jointType = JOINTTYPE::NONE, const std::string& name = std::string(""), unsigned int dof = 0);
			Joint(const Joint& otherJoint);
			virtual ~Joint() = default;

			Joint& operator=(const Joint& otherJoint);
			
			bool	setLimitPos(const Math::VectorX& lower, const Math::VectorX& upper);
			bool	setLimitVel(const Math::VectorX& lower, const Math::VectorX& upper);
			bool	setLimitAcc(const Math::VectorX& lower, const Math::VectorX& upper);
			bool    setLimitJerk(const Math::VectorX& lower, const Math::VectorX& upper);
			bool	setLimitInput(const Math::VectorX& lower, const Math::VectorX& upper);
			bool	setConstSpring(const Math::VectorX& values);
			bool	setConstDamper(const Math::VectorX& values);
			bool	setConstFriction(const Math::VectorX& values);

			const JOINTTYPE&	getJointType() const { return _jointType; }
			const std::string&	getName() const			{ return _name; }
			unsigned int		getDOF() const			{ return _dof; }

			const Math::VectorX&	getLimitPosLower() const	{ return _LimitPosLower; }
			const Math::VectorX&	getLimitPosUpper() const	{ return _LimitPosUpper; }
			const Math::VectorX&	getLimitVelLower() const	{ return _LimitVelLower; }
			const Math::VectorX&	getLimitVelUpper() const	{ return _LimitVelUpper; }
			const Math::VectorX&	getLimitAccLower() const	{ return _LimitAccLower; }
			const Math::VectorX&	getLimitAccUpper() const	{ return _LimitAccUpper; }
			const Math::VectorX&	getLimitJerkUpper() const	{ return _LimitJerkUpper; }
			const Math::VectorX&	getLimitJerkLower() const	{ return _LimitJerkLower; }
			const Math::VectorX&	getLimitInputUpper() const  { return _LimitInputUpper; }
			const Math::VectorX&	getLimitInputLower() const  { return _LimitInputLower; }
			const Math::VectorX&	getConstSpring() const		{ return _ConstantSpring; }
			const Math::VectorX&	getConstDamper() const		{ return _ConstantDamper; }
			const Math::VectorX&	getConstFriction() const	{ return _ConstantFriction; }

			
			virtual JointPtr copy() const = 0;

			virtual void updateTransform(State::JointState& state) const = 0;
			virtual void updateJacobian(State::JointState& state) const = 0;
			virtual void updateJacobianDot(State::JointState& state) const = 0;

		protected:
			bool setName(const std::string& otherName);

			JOINTTYPE _jointType;

			///	Unique string representation of joint.
			std::string		_name;
			///	Degree of freedom of joint. It can not be changed after construction.
			unsigned int	_dof;
			
			Math::VectorX				_LimitPosLower;
			Math::VectorX				_LimitPosUpper;
			Math::VectorX				_LimitVelLower;
			Math::VectorX				_LimitVelUpper;
			Math::VectorX				_LimitAccLower;
			Math::VectorX				_LimitAccUpper;
			Math::VectorX				_LimitJerkLower;
			Math::VectorX				_LimitJerkUpper;
			Math::VectorX				_LimitInputLower;
			Math::VectorX				_LimitInputUpper;

			Math::VectorX				_ConstantSpring;
			Math::VectorX				_ConstantDamper;
			Math::VectorX				_ConstantFriction;
		};

	}
}