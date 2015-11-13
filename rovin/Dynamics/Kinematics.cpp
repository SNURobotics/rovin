#include "System.h"
#include "State.h"

#include <list>
#include <vector>

#include <rovin/Math/Constant.h>
#include <rovin/Math/LinearAlgebra.h>

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
					T *= iter->_sj * _jointptr[iter->_joint]->getTransform(state.getJointState(iter->_joint).q, iter->_isReverse) * iter->_je;
				}
				f.block<6, 1>(i * 6, 0) = SE3::Log(T);
			}

			return f;
		}

		Math::MatrixX System::Closedloop_Constraint_Jacobian(State& state, const RETURN_STATE& return_state = SYSTEMJOINT)
		{
			unsigned int column = state.getDof(return_state);
			MatrixX J(_closedloop.size() * 6, column);
			SE3 T, jointT;
			int i = 0;
			for (list< list< _CONN >>::const_iterator coniter = _closedloop.begin(); coniter != _closedloop.end(); coniter++, i++)
			{
				T = SE3();
				for (list< _CONN >::const_reverse_iterator iter = (*coniter).rbegin(); iter != (*coniter).rend(); iter++)
				{
					T = iter->_je * T;
					MatrixX temp_J = SE3::invAd(T) * _jointptr[iter->_joint]->getJacobian(state.getJointState(iter->_joint).q, iter->_isReverse);
					state.writeColumns(J, temp_J, 6 * i, iter->_joint, return_state);

					T = iter->_sj * _jointptr[iter->_joint]->getTransform(state.getJointState(iter->_joint).q, iter->_isReverse) * T;
				}
			}
			return J;
		}

		void System::Solve_Closedloop_Constraint(State& state)
		{
			if (_closedloop.size() == 0) return;

			VectorX S, dtheta;
			while ((S = Closedloop_Constraint_Function(state)).norm() >= Eigen::NumTraits<Real>::dummy_precision())
			{
				MatrixX J = Closedloop_Constraint_Jacobian(state, System::PASSIVEJOINT);
				state.addPassiveJoint_q(-pinv(J) * S);
			}
		}

		void System::Forward_Kinematics(State & state)
		{
			Solve_Closedloop_Constraint(state);
			for (unsigned i = 0; i < _BFSIdx.size(); i++)
			{
				state.getLinkState(_BFSIdx[i]._elink).T =
					state.getLinkState(_BFSIdx[i]._slink).T
					* _BFSIdx[i]._sj
					* _jointptr[_BFSIdx[i]._joint]->getTransform(state.getJointState(_BFSIdx[i]._joint).q, _BFSIdx[i]._isReverse)
					* _BFSIdx[i]._je;
			}
		}

		void System::Forward_Diff_Kinematics(State & state)
		{
			//	update qdot
			for (unsigned i = 0; i < _BFSIdx.size(); i++)
			{
				state.getLinkState(_BFSIdx[i]._elink).V =
					Math::SE3::Ad(state.getLinkState(_BFSIdx[i]._elink).T.inverse()*state.getLinkState(_BFSIdx[i]._slink).T)
					* state.getLinkState(_BFSIdx[i]._slink).V
					+ Math::SE3::invAd(_BFSIdx[i]._je)
					* _jointptr[_BFSIdx[i]._joint]->getJacobian(state.getJointState(_BFSIdx[i]._joint).q, _BFSIdx[i]._isReverse) * state.getJointState(_BFSIdx[i]._joint).qdot;
			}
		}

	}
}