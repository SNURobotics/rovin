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
}