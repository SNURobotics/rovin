#include "Kinematics.h"

#include <rovin/Math/Constant.h>
#include <rovin/Math/LinearAlgebra.h>

using namespace std;
using namespace rovin::Math;
using namespace rovin::Model;

namespace rovin
{
	VectorX Kinematics::computeClosedLoopConstraintFunction(const Assembly& assem, const State& state)
	{
		VectorX f(assem._ClosedLoopConstraint.size() * 6);
		SE3 T;
		for (unsigned int i = 0; i < assem._ClosedLoopConstraint.size(); i++)
		{
			T = SE3();
			for (unsigned int j = 0; j < assem._ClosedLoopConstraint[i].size(); j++)
			{
				unsigned int mateIdx = assem._ClosedLoopConstraint[i][j].first;
				if (assem._ClosedLoopConstraint[i][j].second == JointDirection::REGULAR)
				{
					T *= SE3::multiply(assem._Mate[mateIdx]._Tmj,
									   assem._Mate[mateIdx]._joint->getTransform(state.getJointStateByMateIndex(mateIdx)._q, false),
									   assem._Mate[mateIdx]._Tja);
				}
				else
				{
					T *= SE3::multiply(assem._Mate[mateIdx]._InvTja,
									   assem._Mate[mateIdx]._joint->getTransform(state.getJointStateByMateIndex(mateIdx)._q, true),
									   assem._Mate[mateIdx]._InvTmj);
				}
			}
			f.block<6, 1>(i * 6, 0) = SE3::Log(T);
		}
		return f;
	}

	MatrixX  Kinematics::computeClosedLoopConstraintJacobian(const Assembly& assem, const State& state, const State::RETURN_STATE& return_state)
	{
		MatrixX J(assem._ClosedLoopConstraint.size() * 6, state.returnDof(return_state));
		SE3 T;
		for (unsigned int i = 0; i < assem._ClosedLoopConstraint.size(); i++)
		{
			T = SE3();
			for (int j = assem._ClosedLoopConstraint[i].size() - 1; j >= 0; j--)
			{
				unsigned int mateIdx = assem._ClosedLoopConstraint[i][j].first;
				if (assem._ClosedLoopConstraint[i][j].second == JointDirection::REGULAR)
				{
					T = assem._Mate[mateIdx]._Tja * T;
					state.writeReturnMatrix(J,
											SE3::invAd(T)*assem._Mate[mateIdx]._joint->getJacobian(state.getJointStateByMateIndex(mateIdx)._q, false),
											6 * i, state.getJointIndexByMateIndex(mateIdx), return_state);
					T = SE3::multiply(assem._Mate[mateIdx]._Tmj,
									  assem._Mate[mateIdx]._joint->getTransform(state.getJointStateByMateIndex(mateIdx)._q, false),
									  T);
				}
				else
				{
					T = assem._Mate[mateIdx]._InvTmj * T;
					state.writeReturnMatrix(J,
											SE3::invAd(T)*assem._Mate[mateIdx]._joint->getJacobian(state.getJointStateByMateIndex(mateIdx)._q, true),
											6 * i, state.getJointIndexByMateIndex(mateIdx), return_state);
					T = SE3::multiply(assem._Mate[mateIdx]._InvTja,
									  assem._Mate[mateIdx]._joint->getTransform(state.getJointStateByMateIndex(mateIdx)._q, true),
									  T);
				}
			}
		}
		return J;
	}

	void Kinematics::solveClosedLoopConstraint(const Assembly& assem, State& state)
	{
		if (assem._ClosedLoopConstraint.size() == 0) return;

		VectorX S, dtheta;
		while ((S = Kinematics::computeClosedLoopConstraintFunction(assem, state)).squaredNorm() >= RealEps)
		{
			state.addPassiveJointq(-pInv(Kinematics::computeClosedLoopConstraintJacobian(assem, state, State::PASSIVEJOINT))*S);
		}
	}
	void Kinematics::solveForwardKinematics(const Model::Assembly & assem, Model::State & state)
	{
		for (unsigned int i = 0; i < assem._Tree.size(); i++)
		{
			unsigned int mateIdx = assem._Tree[i].first;
			const Assembly::Mate&	mate = assem._Mate[mateIdx];
			SE3 T_pc;
			//	update transform and local velocity of each joint
			mate._joint->updateForwardKinematics(state.getJointStateByMateIndex(mateIdx), assem._Tree[i].second, true, true);
			//	update link state by propagation
			if (assem._Tree[i].second == Model::REGULAR)
			{
				//	link position update
				T_pc = mate._Tmj * state.getJointStateByMateIndex(mateIdx)._T[0] * mate._Tja;
				state.getLinkState(mate._actionLinkIdx)._T
					= state.getLinkState(mate._mountLinkIdx)._T * T_pc;
				//	link velocity update (currently blocked to measure performance)
				//state.getLinkState(mate._actionLinkIdx)._V
				//	= SE3::invAd(T_pc)*state.getLinkState(mate._mountLinkIdx)._V
				//	+ SE3::invAd(mate._Tja)*state.getJointStateByMateIndex(mateIdx)._v;
			}
			else	// reversed case (parent link is action link)
			{
				//	link position update
				T_pc = mate._InvTja * state.getJointStateByMateIndex(mateIdx)._T[0] * mate._InvTmj;
				state.getLinkState(mate._mountLinkIdx)._T
					= state.getLinkState(mate._actionLinkIdx)._T * T_pc;
				//	link velocity update (currently blocked to measure performance)
				//state.getLinkState(mate._mountLinkIdx)._V
				//	= SE3::invAd(T_pc)*state.getLinkState(mate._actionLinkIdx)._V
				//	+ SE3::Ad(mate._Tmj)*state.getJointStateByMateIndex(mateIdx)._v;
			}
		}
	}
}