#include "Joint.h"

#include <rovin/utils/Checker.h>

namespace rovin {
	namespace Model {
		Joint::Joint(const JOINTTYPE& jointType, const std::string & name, unsigned int dof)
			:_name((utils::checkName(name) ? (name) : (assert(0 && "Invalid name."), "")))
			, _jointType(jointType)
			, _dof((dof>=0)?(dof): (assert(0 && "DOF of joint cannot be negative"), 0))
		{
			if (dof > 0)
			{
				_LimitPosLower = Math::VectorX::Ones(dof) * std::numeric_limits<Math::Real>::min();
				_LimitPosUpper = Math::VectorX::Ones(dof) * std::numeric_limits<Math::Real>::max();

				_LimitVelLower = Math::VectorX::Ones(dof) * std::numeric_limits<Math::Real>::min();
				_LimitVelUpper = Math::VectorX::Ones(dof) * std::numeric_limits<Math::Real>::max();

				_LimitAccLower = Math::VectorX::Ones(dof) * std::numeric_limits<Math::Real>::min();
				_LimitAccUpper = Math::VectorX::Ones(dof) * std::numeric_limits<Math::Real>::max();

				_LimitInputLower = Math::VectorX::Ones(dof) * std::numeric_limits<Math::Real>::min();
				_LimitInputUpper = Math::VectorX::Ones(dof) * std::numeric_limits<Math::Real>::max();

				_ConstantSpring = Math::VectorX::Zero(dof);
				_ConstantDamper = Math::VectorX::Zero(dof);
				_ConstantFriction = Math::VectorX::Zero(dof);
			}
			return;
		}
		Joint::Joint(const Joint & otherJoint)
		{
			*this = otherJoint;
		}
		Joint & Joint::operator=(const Joint & otherJoint)
		{
			if (this != &otherJoint)
			{
				_name = /*std::string("_") +*/ otherJoint.getName();
				_dof = otherJoint.getDOF();

				_LimitPosLower = otherJoint.getLimitPosLower();
				_LimitPosUpper = otherJoint.getLimitPosUpper();
				_LimitVelLower = otherJoint.getLimitVelLower();
				_LimitVelUpper = otherJoint.getLimitVelUpper();
				_LimitAccLower = otherJoint.getLimitAccLower();
				_LimitAccUpper = otherJoint.getLimitAccupper();

				_ConstantSpring = otherJoint.getConstSpring();
				_ConstantDamper = otherJoint.getConstDamper();
				_ConstantFriction = otherJoint.getConstFriction();
			}
			return *this;
		}

		bool Joint::setLimitPos(const Math::VectorX & lower, const Math::VectorX & upper)
		{
			assert((lower.size() == _dof&&upper.size() == _dof) && "Size of limit must equal to DOF of joint");
			for (unsigned i = 0; i < _dof; i++)
				assert(lower[i] <= upper[i] && "Lower limit should be smaller than upper one.");
			_LimitPosLower = lower;
			_LimitPosUpper = upper;
			return true;
		}

		bool Joint::setLimitVel(const Math::VectorX & lower, const Math::VectorX & upper)
		{
			assert((lower.size() == _dof&&upper.size() == _dof) && "Size of limit must equal to DOF of joint");
			for (unsigned i = 0; i < _dof; i++)
				assert(lower[i] <= upper[i] && "Lower limit should be smaller than upper one.");
			_LimitVelLower = lower;
			_LimitVelUpper = upper;
			return true;
		}

		bool Joint::setLimitAcc(const Math::VectorX & lower, const Math::VectorX & upper)
		{
			assert((lower.size() == _dof&&upper.size() == _dof) && "Size of limit must equal to DOF of joint");
			for (unsigned i = 0; i < _dof; i++)
				assert(lower[i] <= upper[i] && "Lower limit should be smaller than upper one.");
			_LimitAccLower = lower;
			_LimitAccUpper = upper;
			return true;
		}

		bool Joint::setLimitInput(const Math::VectorX & lower, const Math::VectorX & upper)
		{
			assert((lower.size() == _dof&&upper.size() == _dof) && "Size of limit must equal to DOF of joint");
			for (unsigned i = 0; i < _dof; i++)
				assert(lower[i] <= upper[i] && "Lower limit should be smaller than upper one.");
			_LimitInputLower = lower;
			_LimitInputUpper = upper;
			return false;
		}

		bool Joint::setConstSpring(const Math::VectorX & values)
		{
			assert(values.size() == _dof && "Size of constant must equal to DOF of joint");
			_ConstantSpring = values;
			return true;
		}

		bool Joint::setConstDamper(const Math::VectorX & values)
		{
			assert(values.size() == _dof && "Size of constant must equal to DOF of joint");
			_ConstantDamper = values;
			return true;
		}

		bool Joint::setConstFriction(const Math::VectorX & values)
		{
			assert(values.size() == _dof && "Size of constant must equal to DOF of joint");
			_ConstantFriction = values;
			return true;
		}

		bool Joint::setName(const std::string & otherName)
		{
			if (utils::checkName(otherName) == true)
			{
				_name = otherName;
				return true;
			}
			else
				return false;
		}

	}
}