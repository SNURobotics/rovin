#include "System.h"

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
			utils::Log(_connectstate, "이 함수를 사용하기 위해서는 state에 연결되어 있으면 안됩니다.", true);
			ConnectState(state);
			VectorX f = Closedloop_Constraint_Function();
			DisconnectState();
			return f;
		}

		VectorX System::Closedloop_Constraint_Function() const
		{
			utils::Log(_connectstate, "이 함수를 사용하기 위해서는 state에 연결되어 있어야 합니다.", true);

			/// \todo 조건이 없는 경우는?
			VectorX f(_closedloop.size() * 6);
			SE3 T, jointT;
			int i = 0;
			for (list< list< _CONN >>::const_iterator coniter = _closedloop.begin(); coniter != _closedloop.end(); coniter++, i ++)
			{
				T = SE3();
				for (list< _CONN >::const_iterator iter = (*coniter).begin(); iter != (*coniter).end(); iter++)
				{
					if (iter->_direction)
					{
						T *= iter->_sj * _jointptr[iter->_joint]->getTransform(_jointstate[iter->_joint]->q) * iter->_je;
					}
					else
					{
						T *= iter->_sj * _jointptr[iter->_joint]->getTransform(_jointstate[iter->_joint]->q).inverse() * iter->_je;
					}
				}
				f.block(i * 6, 0, 6, 1) = SE3::Log(T);
			}

			return f;
		}

		Math::MatrixX System::Closedloop_Constraint_Jacobian(State& state)
		{
			utils::Log(_connectstate, "이 함수를 사용하기 위해서는 state에 연결되어 있으면 안됩니다.", true);
			ConnectState(state);
			MatrixX J = Closedloop_Constraint_Jacobian();
			DisconnectState();
			return J;
		}

		Math::MatrixX System::Closedloop_Constraint_Jacobian() const
		{
			MatrixX J(_closedloop.size() * 6, _state->getActiveJointDof());
			SE3 T, jointT;
			int i = 0;
			for (list< list< _CONN >>::const_iterator coniter = _closedloop.begin(); coniter != _closedloop.end(); coniter++, i++)
			{
				T = SE3();
				for (list< _CONN >::const_reverse_iterator iter = (*coniter).rbegin(); iter != (*coniter).rend(); iter--)
				{
					T = iter->_je * T;
					if (_activejoint[iter->_joint])
					{
						MatrixX temp_J = SE3::invAd(T) * _jointptr[iter->_joint]->getJacobian(_jointstate[iter->_joint]->q);
						if (!iter->_direction)
						{
							temp_J *= -1;
						}
						for (int j = 0; j < _jointstate[iter->_joint]->dof; j++)
						{
							J.block(6 * i, _activejoint_index[iter->_joint] + j, 6, 1) = temp_J.col(j);
						}
					}
					if (iter->_direction)
					{
						T = iter->_sj * _jointptr[iter->_joint]->getTransform(_jointstate[iter->_joint]->q) * T;
					}
					else
					{
						T = iter->_sj * _jointptr[iter->_joint]->getTransform(_jointstate[iter->_joint]->q).inverse() * T;
					}
				}
			}
			return J;
		}
	}
}