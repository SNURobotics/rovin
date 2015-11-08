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
#include <rovin/Math/Inertia.h>
#include <rovin/Math/LieGroup.h>

namespace rovin
{
	namespace Model
	{
		class Link;

		bool checkName(std::string);

		/**
		*	\class Joint
		*	\brief Joint class specify kinematic information about general mechanical joints.
		*/

		class Joint
		{
			friend class Assembly;

		public:
			typedef double						real;
			typedef Eigen::Matrix<real, -1, 1>	vec;
			typedef Eigen::Matrix<real, 6, -1>	jacobian;
			typedef std::weak_ptr<Link>			linkPtr;
			typedef std::weak_ptr<Joint>		jointPtr_weak;
			typedef std::shared_ptr<Joint>		jointPtr_shared;
			typedef rovin::Math::SE3			SE3;
			
			Joint(const std::string& name = std::string(""), unsigned int dof = 0);
			Joint(const Joint& otherJoint);
			virtual ~Joint() = default;

			Joint& operator=(const Joint& otherJoint);
			const jointPtr_shared	copy();

			bool	setLimitPos(const vec& lower, const vec& upper);
			bool	setLimitVel(const vec& lower, const vec& upper);
			bool	setLimitAcc(const vec& lower, const vec& upper);
			bool	setLimitInput(const vec& lower, const vec& upper);
			bool	setConstSpring(const vec& values);
			bool	setConstDamper(const vec& values);
			bool	setConstFriction(const vec& values);

			const std::string&	getName() const		{ return _name; }
			unsigned int		getDOF() const		{ return _dof; }
			const linkPtr&	getMountLinkPtr() const{ return _mountLinkPtr; }
			const linkPtr&	getActionLinkPtr() const { return _actionLinkPtr; }
			const SE3&		getMountLinkToJointFrame() const	{ return _mountLinkFrame; }
			const SE3&		getActionLinkToJointFrame()	const { return _actionLinkFrame; }
			const vec&	getLimitPosLower() const	{ return _LimitPosLower; }
			const vec&	getLimitPosUpper() const	{ return _LimitPosUpper; }
			const vec&	getLimitVelLower() const	{ return _LimitVelLower; }
			const vec&	getLimitVelUpper() const	{ return _LimitVelUpper; }
			const vec&	getLimitAccLower() const	{ return _LimitAccLower; }
			const vec&	getLimitAccupper() const	{ return _LimitAccUpper; }
			const vec&	getConstSpring() const		{ return _ConstantSpring; }
			const vec&	getConstDamper() const		{ return _ConstantDamper; }
			const vec&	getConstFriction() const	{ return _ConstantFriction; }

			

			///	Return static jacobian matrix (6 by DOF). Derived class should implement this.
			//virtual const jacobian& getJacobianStatic(const vec& state) const = 0;
			//virtual const jacobian& getJacobianTimeDerivStatic(const vec& state) const = 0;


		protected:
			//bool setName(const std::string& otherName);
			bool addMountLink(const linkPtr& mountLinkPtr, const SE3& frameLinkToJoint = SE3());
			bool addActionLink(const linkPtr& actionLinkPtr, const SE3& frameLinkToJoint = SE3());
			bool removeMountLink();
			bool removeActionLink();

			///	Unique string representation of joint.
			std::string		_name;
			///	Degree of freedom of joint. It can not be changed after construction.
			unsigned int	_dof;

			///	Pointer to mount Link
			linkPtr			_mountLinkPtr;
			///	Frame of this w.r.t. mount link frame.
			Math::SE3		_mountLinkFrame;
			///	Pointer to child link.
			linkPtr			_actionLinkPtr;
			///	Frame of this w.r.t. child link frame
			Math::SE3		_actionLinkFrame;
			
			vec				_LimitPosLower;
			vec				_LimitPosUpper;
			vec				_LimitVelLower;
			vec				_LimitVelUpper;
			vec				_LimitAccLower;
			vec				_LimitAccUpper;
			vec				_LimitInputLower;
			vec				_LimitInputUpper;

			vec				_ConstantSpring;
			vec				_ConstantDamper;
			vec				_ConstantFriction;
		};

	}
}