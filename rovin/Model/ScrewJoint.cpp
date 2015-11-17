#include "ScrewJoint.h"

#include <iostream>
using namespace std;

namespace rovin {
	namespace Model {
		ScrewJoint::ScrewJoint(const std::string & name, unsigned int dof, const Math::MatrixX & axes)
			:Joint(name, dof)
		{
			if (axes.rows() == 6 && axes.cols() == _dof)
			{
				_axes = axes;
				//normalizeAxes();
			}
			else if (axes.rows() == 3 && axes.cols() == _dof)
			{
				_axes.resize(6, dof);
				_axes.setZero();
				_axes.topRows<3>() = axes;
				_axes.bottomRows<3>().setZero();
				//normalizeAxes();
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
			_axes.block<3, 1>(0, index) = axis;
			_axes.block<3, 1>(3, index).setZero();
		}

		void ScrewJoint::setAxis(const Math::Vector6 & axis, unsigned int index)
		{
			_axes.col(index) = axis;
			//_axes.block<3, 1>(0, index) = axis.head<3>().normalized();
			//_axes.block<3, 1>(3, index) = axis.tail<3>();
		}

		void ScrewJoint::normalizeAxes()
		{
			for (unsigned i = 0; i < _dof; i++)
			{
				_axes.block<3, 1>(0, i).normalize();
				_axes.block<3, 1>(3, i).normalize();
			}
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

		Math::SE3 ScrewJoint::getTransform(const Math::VectorX & state, bool isReversed) const
		{
			if (_dof == 0)
				return Math::SE3();
			else if (_dof != state.size())
			{
				assert(0 && "Input dimension mismatch.");
				return Math::SE3();
			}

			//	In most case, it has regular direction and single DOF.
			Math::SE3 T = Math::SE3::Exp(_axes.col(0), state[0]);
			if (!isReversed)
			{
				for (unsigned int i = 1; i < _dof; i++)
					T *= Math::SE3::Exp(_axes.col(i), state[i]);
			}
			else
			{
				T = Math::SE3::Exp(_axes.col(_dof-1), -state[0]);
				for (unsigned int i = _dof - 2; i >= 0; i--)
					T *= Math::SE3::Exp(_axes.col(i), -state[i]);
			}
			
			return T;
		}

		Math::se3 ScrewJoint::getVelocity(const Math::VectorX & state, bool isReversed) const
		{
			//	TODO: implement this. (It should accept joint velocity as input)
			Math::se3	localVel;
			Math::SE3	localFrame;
			//if (!isReversed)
			//{
			//	localVel = _axes.col(_dof - 1) * state[_dof - 1];
			//	for (unsigned i = _dof - 1; i > 0; i--)
			//	{
			//		localFrame = Math::SE3::Exp(_axes.col(i), state[i]) * localFrame;
			//		localVel += Math::SE3::invAd(localFrame);
			//	}
			//}
			//else
			//{
			//	localVel = _axes.col(0) * state[0];
			//	for (unsigned i = 1; i < _dof; i++)
			//	{

			//	}
			//}


			return localVel;
		}

		Math::Matrix6X ScrewJoint::getJacobian(const Math::VectorX & state, bool isReversed) const
		{
			Math::MatrixX	J(6, _dof);
			Math::SE3 T;
			if (!isReversed)
			{
				J.col(_dof - 1) = _axes.col(_dof - 1);
				for (unsigned i = _dof - 1; i > 0; i--)
				{
					T = Math::SE3::Exp(_axes.col(i), state[i]) * T;
					J.col(i - 1) = Math::SE3::invAd(T) * _axes.col(i - 1);
				}
			}
			else
			{
				J.col(0) = -_axes.col(0);
				for (unsigned i = 0; i < _dof - 1; i++)
				{
					T *= Math::SE3::Exp(_axes.col(i), state[i]);
					J.col(i + 1) = - Math::SE3::Ad(T) * _axes.col(i + 1);
				}
			}
			return J;
		}

		Math::Matrix6X ScrewJoint::getJacobianDot(const Math::VectorX & state) const
		{
			//	TODO: implement this function
			return Math::MatrixX();
		}

	}
}

