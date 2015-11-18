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

		enum JointDirection {REGULAR, REVERSE};

		/**
		*	\class Joint
		*	\brief Joint class specify kinematic information about general mechanical joints.
		*/
		class Joint
		{
			friend class Assembly;

		public:
			Joint(const std::string& name = std::string(""), unsigned int dof = 0);
			Joint(const Joint& otherJoint);
			virtual ~Joint() = default;

			Joint& operator=(const Joint& otherJoint);
			
			bool	setLimitPos(const Math::VectorX& lower, const Math::VectorX& upper);
			bool	setLimitVel(const Math::VectorX& lower, const Math::VectorX& upper);
			bool	setLimitAcc(const Math::VectorX& lower, const Math::VectorX& upper);
			bool	setLimitInput(const Math::VectorX& lower, const Math::VectorX& upper);
			bool	setConstSpring(const Math::VectorX& values);
			bool	setConstDamper(const Math::VectorX& values);
			bool	setConstFriction(const Math::VectorX& values);

			const std::string&	getName() const		{ return _name; }
			unsigned int		getDOF() const		{ return _dof; }

			const Math::VectorX&	getLimitPosLower() const	{ return _LimitPosLower; }
			const Math::VectorX&	getLimitPosUpper() const	{ return _LimitPosUpper; }
			const Math::VectorX&	getLimitVelLower() const	{ return _LimitVelLower; }
			const Math::VectorX&	getLimitVelUpper() const	{ return _LimitVelUpper; }
			const Math::VectorX&	getLimitAccLower() const	{ return _LimitAccLower; }
			const Math::VectorX&	getLimitAccupper() const	{ return _LimitAccUpper; }
			const Math::VectorX&	getConstSpring() const		{ return _ConstantSpring; }
			const Math::VectorX&	getConstDamper() const		{ return _ConstantDamper; }
			const Math::VectorX&	getConstFriction() const	{ return _ConstantFriction; }

			
			virtual JointPtr copy() const = 0;
			virtual Math::SE3	getTransform(const Math::VectorX& state, bool isReversed = false) const = 0;
			virtual Math::se3	getVelocity(const Math::VectorX& state, bool isReversed = false) const = 0;
			///	Return static jacobian matrix (6 by DOF). Derived class should implement this.
			virtual Math::Matrix6X getJacobian(const Math::VectorX& state, bool isReversed = false) const = 0;
			virtual Math::Matrix6X getJacobianDot(const Math::VectorX& state) const = 0;

			virtual void	updateForwardKinematics(State::JointState& state, JointDirection direction = REGULAR, bool position = true, bool velocity = false, bool acceleration = false) const = 0;


		protected:
			bool setName(const std::string& otherName);

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
			Math::VectorX				_LimitInputLower;
			Math::VectorX				_LimitInputUpper;

			Math::VectorX				_ConstantSpring;
			Math::VectorX				_ConstantDamper;
			Math::VectorX				_ConstantFriction;
		};

	}
}