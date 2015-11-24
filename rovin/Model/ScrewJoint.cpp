#include "ScrewJoint.h"

#include <iostream>
using namespace std;

namespace rovin {
	namespace Model {
		ScrewJoint::ScrewJoint(const std::string & name, unsigned int dof, const Math::MatrixX & axes)
			:Joint(JOINTTYPE::SCREWJOINT, name,  dof)
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

		//void ScrewJoint::adjointAxes(const Math::SE3 & TransformFromJoint)
		//{
		//	_axes = Math::SE3::Ad(TransformFromJoint) * _axes;
		//}

		ScrewJoint & rovin::Model::ScrewJoint::operator=(const ScrewJoint & otherJoint)
		{
			if (this != &otherJoint)
			{
				Joint::operator=(otherJoint);
				_axes = otherJoint.getAxes();
			}

			return *this;
		}

		JointPtr ScrewJoint::copy() const
		{
			return JointPtr(new ScrewJoint(*this));
		}

		void ScrewJoint::updateTransform(State::JointState& state) const
		{
			const Math::VectorX &q = state.getq();
			state._T[0] = Math::SE3::Exp(_axes.col(0), q[0]);
			for (unsigned int i = 1; i < _dof; i++)
				state._T[i] = Math::SE3::Exp(_axes.col(i), q[i]) * state._T[i - 1];
		}

		void ScrewJoint::updateJacobian(State::JointState& state) const
		{
			state._J.col(0) = _axes.col(0);
			for (unsigned int i = 1; i < _dof; i++)
				state._J.col(i) = Math::SE3::Ad(state._T[i - 1]) * _axes.col(i);
		}

		void ScrewJoint::updateJacobianDot(State::JointState& state) const
		{
			//TODO
		}
	}
}

