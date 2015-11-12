#include "System.h"
#include "State.h"

#include <list>
#include <vector>

#include <rovin/Math/Constant.h>

using namespace std;
using namespace rovin::Math;

namespace rovin
{
	namespace Dynamics
	{
		VectorX System::Closedloop_Constraint_Function(State& state)
		{
			VectorX f(_closedloop.size() * 6);
			SE3 T, jointT;
			int i = 0;
			for (list< list< _CONN >>::const_iterator coniter = _closedloop.begin(); coniter != _closedloop.end(); coniter++, i++)
			{
				T = SE3();
				for (list< _CONN >::const_iterator iter = (*coniter).begin(); iter != (*coniter).end(); iter++)
				{
					if (iter->_direction)
					{
						T *= iter->_sj * _jointptr[iter->_joint]->getTransform(state.getJointState(iter->_joint).q) * iter->_je;
					}
					else
					{
						T *= iter->_sj * _jointptr[iter->_joint]->getTransform(state.getJointState(iter->_joint).q).inverse() * iter->_je;
					}
				}
				f.block(i * 6, 0, 6, 1) = SE3::Log(T);
			}
			
			return f;
		}

		Math::MatrixX System::Closedloop_Constraint_Jacobian(State& state, const RETURN_STATE& return_state = SYSTEMJOINT)
		{
			unsigned int column = state.getReturnDof(return_state);
			MatrixX J(_closedloop.size() * 6, column);
			SE3 T, jointT;
			int i = 0;
			for (list< list< _CONN >>::const_iterator coniter = _closedloop.begin(); coniter != _closedloop.end(); coniter++, i++)
			{
				T = SE3();
				for (list< _CONN >::const_reverse_iterator iter = (*coniter).rbegin(); iter != (*coniter).rend(); iter--)
				{
					T = iter->_je * T;
					MatrixX temp_J = SE3::invAd(T) * _jointptr[iter->_joint]->getJacobian(state.getJointState(iter->_joint).q);
					if (!iter->_direction)
					{
						temp_J *= -1;
					}
					state.makeReturnMatrix(J, temp_J, 6 * i, iter->_joint, return_state);

					if (iter->_direction)
					{
						T = iter->_sj * _jointptr[iter->_joint]->getTransform(state.getJointState(iter->_joint).q) * T;
					}
					else
					{
						T = iter->_sj * _jointptr[iter->_joint]->getTransform(state.getJointState(iter->_joint).q).inverse() * T;
					}
				}
			}
			return J;
		}

		void System::Solve_Closedloop_Constraint(State& state)
		{
			if (_closedloop.size() == 0) return;

			VectorX S, dtheta;
			while ((S = Closedloop_Constraint_Function(state)).norm() >= Eigen::NumTraits<double>::dummy_precision())
			{
				MatrixX J = Closedloop_Constraint_Jacobian(state, System::PASSIVEJOINT);
				state.setPassiveJoint_q(-J.inverse()*S);
			}
		}
	}
}