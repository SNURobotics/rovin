#include "ScrewJoint.h"

namespace rovin {
	namespace Model {
		ScrewJoint::ScrewJoint(const std::string & name, unsigned int dof, const Math::MatrixX & axes)
			:Joint(name, dof)
		{
			if (axes.rows() == 6 && axes.cols() == _dof)
				_axes = axes;
			else if (axes.rows() == 3 && axes.cols() == _dof)
			{
				_axes.resize(6, dof);
				_axes.topRows<3>() = axes;
			}
			else
			{
				_axes.resize(6, dof);
				for (unsigned int i = 0; i < _dof; i++)
				{
					if (i % 3 == 0)
						_axes.block<3, 1>(0, i) = Math::Vector3::UnitZ();
					else if (i % 3 == 1)
						_axes.block<3, 1>(0, i) = Math::Vector3::UnitY();
					else
						_axes.block<3, 1>(0, i) = Math::Vector3::UnitX();
				}
				_axes.bottomRows<3>().setZero();
			}
			return;
		}

		ScrewJoint::ScrewJoint(const ScrewJoint & otherJoint)
			:Joint(otherJoint), _axes(otherJoint.getAxes())
		{
			//	do nothing
		}

		void ScrewJoint::setAxis(const Math::Vector3 & axis, unsigned int index)
		{
			_axes.block<3, 1>(0, index) = axis.normalized();
			_axes.block<3, 1>(3, index).setZero();
		}

		void ScrewJoint::setAxis(const Math::Vector6 & axis, unsigned int index)
		{
			_axes.block<3, 1>(0, index) = axis.head<3>().normalized();
			_axes.block<3, 1>(3, index) = axis.tail<3>();
		}

		ScrewJoint & rovin::Model::ScrewJoint::operator=(const ScrewJoint & otherJoint)
		{
			if (this != &otherJoint)
			{
				Joint::operator=(otherJoint);
				_axes = otherJoint.getAxes();
			}

			return *this;
		}

		std::shared_ptr<Joint> ScrewJoint::copy() const
		{
			return std::shared_ptr<Joint>(new ScrewJoint(*this));
		}

		Math::SE3 ScrewJoint::getTransform(const Math::VectorX & state) const
		{
			if (_dof == 0)
				return Math::SE3();
			Math::SE3 T = Math::SE3::Exp(_axes.col(0), state[0]);
			for (unsigned int i = 1; i < _dof; i++)
				T *= Math::SE3::Exp(_axes.col(i), state[i]);
			return T;
		}

		Math::MatrixX ScrewJoint::getJacobian(const Math::VectorX & state) const
		{
			Math::MatrixX	J(6, _dof);
			J.col(_dof - 1) = _axes.col(_dof - 1);
			Math::SE3 T;
			for (size_t i = _dof - 1; i > 0; i--)
			{
				T = Math::SE3::Exp(_axes.col(i), state[i]) * T;
				J.col(i - 1) = Math::SE3::invAd(T) * _axes.col(i - 1);
			}
			return J;
		}

		Math::MatrixX ScrewJoint::getJacobianDot(const Math::VectorX & state) const
		{
			//	TODO: implement this function
			return Math::MatrixX();
		}

	}
}

