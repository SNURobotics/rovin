#include "Joint.h"

namespace rovin {
	namespace Model {
		Joint::Joint(const std::string & name, unsigned int dof)
			:_name((checkName(name) ? (name) : (assert(0 && "Invalid name."), "")))
			,_dof((dof>=0)?(dof): (assert(0 && "DOF of joint cannot be negative"), 0))
			,_parentLinkPtr(linkPtr())
			,_childLinkPtr(linkPtr())
			,_parentLinkFrame(SE3())
			,_childLinkFrame(SE3())
		{
			if (dof > 0)
			{
				_LimitPosLower = vec::Ones(dof) * std::numeric_limits<real>::min();
				_LimitPosUpper = vec::Ones(dof) * std::numeric_limits<real>::max();

				_LimitVelLower = vec::Ones(dof) * std::numeric_limits<real>::min();
				_LimitVelUpper = vec::Ones(dof) * std::numeric_limits<real>::max();

				_LimitAccLower = vec::Ones(dof) * std::numeric_limits<real>::min();
				_LimitAccUpper = vec::Ones(dof) * std::numeric_limits<real>::max();

				_LimitInputLower = vec::Ones(dof) * std::numeric_limits<real>::min();
				_LimitInputUpper = vec::Ones(dof) * std::numeric_limits<real>::max();

				_ConstantSpring = vec::Zero(dof);
				_ConstantDamper = vec::Zero(dof);
				_ConstantFriction = vec::Zero(dof);
			}
			return;
		}
		Joint::Joint(const Joint & otherJoint)
		{
			*this = otherJoint;
		}
		Joint & Joint::operator=(const Joint & otherJoint)
		{
			_name = std::string("_") + otherJoint.getName();
			_dof = otherJoint.getDOF();

			_parentLinkPtr = linkPtr();
			_parentLinkFrame = otherJoint.getParentLinkToJointFrame();
			_childLinkPtr = linkPtr();
			_childLinkFrame = otherJoint.getChildLinkToJointFrame();

			_LimitPosLower = otherJoint.getLimitPosLower();
			_LimitPosUpper = otherJoint.getLimitPosUpper();
			_LimitVelLower = otherJoint.getLimitVelLower();
			_LimitVelUpper = otherJoint.getLimitVelUpper();
			_LimitAccLower = otherJoint.getLimitAccLower();
			_LimitAccUpper = otherJoint.getLimitAccupper();

			_ConstantSpring = otherJoint.getConstSpring();
			_ConstantDamper = otherJoint.getConstDamper();
			_ConstantFriction = otherJoint.getConstFriction();

			return *this;
		}

		const Joint::jointPtr_shared& Joint::copy()
		{
			return std::shared_ptr<Joint>(new Joint(*this));
		}

		bool Joint::setLimitPos(const vec & lower, const vec & upper)
		{
			assert((lower.size() == _dof&&upper.size() == _dof) && "Size of limit must equal to DOF of joint");
			for (auto i = 0; i < _dof; i++)
				assert(lower[i] <= upper[i] && "Lower limit should be smaller than upper one.");
			_LimitPosLower = lower;
			_LimitPosUpper = upper;
			return true;
		}

		bool Joint::setLimitVel(const vec & lower, const vec & upper)
		{
			assert((lower.size() == _dof&&upper.size() == _dof) && "Size of limit must equal to DOF of joint");
			for (auto i = 0; i < _dof; i++)
				assert(lower[i] <= upper[i] && "Lower limit should be smaller than upper one.");
			_LimitVelLower = lower;
			_LimitVelUpper = upper;
			return true;
		}

		bool Joint::setLimitAcc(const vec & lower, const vec & upper)
		{
			assert((lower.size() == _dof&&upper.size() == _dof) && "Size of limit must equal to DOF of joint");
			for (auto i = 0; i < _dof; i++)
				assert(lower[i] <= upper[i] && "Lower limit should be smaller than upper one.");
			_LimitAccLower = lower;
			_LimitAccUpper = upper;
			return true;
		}

		bool Joint::setLimitInput(const vec & lower, const vec & upper)
		{
			assert((lower.size() == _dof&&upper.size() == _dof) && "Size of limit must equal to DOF of joint");
			for (auto i = 0; i < _dof; i++)
				assert(lower[i] <= upper[i] && "Lower limit should be smaller than upper one.");
			_LimitInputLower = lower;
			_LimitInputUpper = upper;
			return false;
		}

		bool Joint::setConstSpring(const vec & values)
		{
			assert(values.size() == _dof && "Size of constant must equal to DOF of joint");
			_ConstantSpring = values;
			return true;
		}

		bool Joint::setConstDamper(const vec & values)
		{
			assert(values.size() == _dof && "Size of constant must equal to DOF of joint");
			_ConstantDamper = values;
			return true;
		}

		bool Joint::setConstFriction(const vec & values)
		{
			assert(values.size() == _dof && "Size of constant must equal to DOF of joint");
			_ConstantFriction = values;
			return true;
		}

		bool Joint::addParentLink(const linkPtr & parentLinkPtr, const SE3 & frameLinkToJoint)
		{
			_parentLinkPtr = parentLinkPtr;
			_parentLinkFrame = frameLinkToJoint;
			return true;
		}

		bool Joint::addChildLink(const linkPtr & childLinkPtr, const SE3 & frameLinkToJoint)
		{
			_childLinkPtr = childLinkPtr;
			_childLinkFrame = frameLinkToJoint;
			return true;
		}



		bool Joint::removeParentLink()
		{
			_parentLinkPtr = linkPtr();
			return true;
		}

		bool Joint::removeChildLink()
		{
			_childLinkPtr = linkPtr();
			return true;
		}

	}
}