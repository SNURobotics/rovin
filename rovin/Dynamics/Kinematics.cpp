#include "Kinematics.h"

#include <rovin/utils/Diagnostic.h>
#include <rovin/Math/Constant.h>
#include <rovin/Math/Common.h>

using namespace std;
using namespace rovin::Math;
using namespace rovin::Model;

namespace rovin
{
	VectorX Kinematics::computeClosedLoopConstraintFunction(const Assembly& assem, State& state)
	{
		if (assem._ClosedLoopConstraint.size() == 0)
			return VectorX::Zero(6);

		VectorX f(assem._ClosedLoopConstraint.size() * 6);
		SE3 T;
		for (unsigned int i = 0; i < assem._ClosedLoopConstraint.size(); i++)
		{
			T = SE3();
			for (unsigned int j = 0; j < assem._ClosedLoopConstraint[i].size(); j++)
			{
				unsigned int mateIdx = assem._ClosedLoopConstraint[i][j].first;

				T *= assem.getTransform(mateIdx, state.getJointStateByMateIndex(mateIdx), assem._ClosedLoopConstraint[i][j].second);
			}
			f.block<6, 1>(i * 6, 0) = SE3::Log(T);
		}
		return f;
	}

	MatrixX Kinematics::computeClosedLoopConstraintJacobian(const Assembly& assem, State& state, const State::TARGET_JOINT& return_state)
	{
		if (assem._ClosedLoopConstraint.size() == 0)
			return MatrixX::Zero(6, state.getDOF(return_state));

		MatrixX J(assem._ClosedLoopConstraint.size() * 6, state.getDOF(return_state));
		SE3 T;
		for (unsigned int i = 0; i < assem._ClosedLoopConstraint.size(); i++)
		{
			T = SE3();
			for (unsigned int j = 0; j < assem._ClosedLoopConstraint[i].size(); j++)
			{
				unsigned int mateIdx = assem._ClosedLoopConstraint[i][j].first;

				if (assem._Mate[mateIdx]._joint->getDOF() != 0)
				{
					state.writeReturnMatrix(J,
						SE3::Ad(T) * assem.getJacobian(mateIdx, state.getJointStateByMateIndex(mateIdx), assem._ClosedLoopConstraint[i][j].second),
						6 * i,
						state.getJointIndexByMateIndex(mateIdx),
						return_state);
				}

				T *= assem.getTransform(mateIdx, state.getJointStateByMateIndex(mateIdx), assem._ClosedLoopConstraint[i][j].second);
			}
		}
		return J;
	}

	void Kinematics::solveClosedLoopConstraint(const Assembly& assem, State& state)
	{
		if (assem._ClosedLoopConstraint.size() == 0) return;

		VectorX S;
		while ((S = Kinematics::computeClosedLoopConstraintFunction(assem, state)).squaredNorm() >= RealEps)
		{
			state.addJointq(State::PASSIVEJOINT, -pInv(Kinematics::computeClosedLoopConstraintJacobian(assem, state, State::PASSIVEJOINT))*S);
		}
	}

	void Kinematics::solveForwardKinematics(const Assembly & assem, State & state, const unsigned int options)
	{
		solveClosedLoopConstraint(assem, state);

		if (state.getJointReferenceFrame() != JointReferenceFrame::JOINTFRAME)
		{
			state.setInfoUpToDate(State::ALL_INFO, false);
			state.setJointReferenceFrame(JointReferenceFrame::JOINTFRAME);
		}

		if ((options & State::LINKS_POS) && !state.getInfoUpToDate(State::LINKS_POS))
		{
			for (unsigned int i = 0; i < assem._Tree.size(); i++)
			{
				unsigned int mateIdx = assem._Tree[i].first;

				//	update link state by propagation
				state.getLinkState(assem._Mate[mateIdx].getChildLinkIdx(assem._Tree[i].second))._T =
					state.getLinkState(assem._Mate[mateIdx].getParentLinkIdx(assem._Tree[i].second))._T *
					assem.getTransform(mateIdx, state.getJointStateByMateIndex(mateIdx), assem._Tree[i].second);
			}
			state.setInfoUpToDate(State::LINKS_POS);
		}
	}

	Matrix6X Kinematics::computeJacobian(const Assembly& assem, State& state, const string& targetLinkMarkerName, const std::string& referenceLinkMarkerName)
	{
		unsigned int targetLinkIdx = assem.getLinkIndex(targetLinkMarkerName);
		unsigned int referenceLinkIdx;
		bool isMarker = false;

		if (targetLinkIdx == -1)
		{
			targetLinkIdx = assem.getLinkIndexByMarkerName(targetLinkMarkerName);
			utils::Log(targetLinkIdx == -1, "targetLinkMarkerName은 링크의 이름이거나 마커의 이름이어야 합니다.", true);
		}

		if (referenceLinkMarkerName.compare("") == 0)
		{
			referenceLinkIdx = assem._baseLink;
		}
		else
		{
			referenceLinkIdx = assem.getLinkIndex(referenceLinkMarkerName);
			if (referenceLinkIdx == -1)
			{
				referenceLinkIdx = assem.getLinkIndexByMarkerName(referenceLinkMarkerName);
				utils::Log(referenceLinkIdx == -1, "referenceLinkMarkerName은 링크의 이름이거나 마커의 이름이어야 합니다.", true);
				isMarker = true;
			}
		}

		if(!isMarker)
			return computeJacobian(assem, state, targetLinkIdx, referenceLinkIdx);
		return SE3::Ad(assem.getLinkPtr(referenceLinkIdx)->getMarker(referenceLinkMarkerName))*computeJacobian(assem, state, targetLinkIdx, referenceLinkIdx);
	}

	Matrix6X Kinematics::computeJacobian(const Assembly& assem, State& state, unsigned int targetLinkIndex, int referenceLinkIndex)
	{
		return computeTransformNJacobian(assem, state, targetLinkIndex, referenceLinkIndex).second;
	}

	pair< SE3, Matrix6X > Kinematics::computeTransformNJacobian(const Assembly& assem, State& state, unsigned int targetLinkIndex, int referenceLinkIndex)
	{
		solveClosedLoopConstraint(assem, state);

		utils::Log(state.getDOF(State::ACTIVEJOINT) == 0, "Active Joint는 하나 이상이어야 합니다.", true);
		MatrixX J(6, state.getDOF(State::STATEJOINT));
		MatrixX ReturnJ;

		if (referenceLinkIndex == -1) referenceLinkIndex = assem._baseLink;
		unsigned int targetLinkDepth = assem._Depth[targetLinkIndex];
		unsigned int referenceLinkDepteh = assem._Depth[referenceLinkIndex];
		SE3 T;
		list< pair< unsigned int, JointDirection >> targetTrace;
		list< pair< unsigned int, JointDirection >> referenceTrace;
		list< pair< unsigned int, JointDirection >> trace;

		while (targetLinkDepth > referenceLinkDepteh)
		{
			unsigned int mateIdx = assem._Parent[targetLinkIndex].first;
			const Assembly::Mate& mate = assem._Mate[mateIdx];

			targetTrace.push_front(assem._Parent[targetLinkIndex]);

			targetLinkIndex = mate.getParentLinkIdx(assem._Parent[targetLinkIndex].second);
			targetLinkDepth--;
		}
		while (targetLinkDepth < referenceLinkDepteh)
		{
			unsigned int mateIdx = assem._Parent[referenceLinkIndex].first;
			const Assembly::Mate& mate = assem._Mate[mateIdx];

			referenceTrace.push_back(Assembly::reverseDirection(assem._Parent[referenceLinkIndex]));

			referenceLinkIndex = mate.getParentLinkIdx(assem._Parent[referenceLinkIndex].second);
			referenceLinkDepteh--;
		}
		while (targetLinkIndex != referenceLinkIndex)
		{
			{
				unsigned int mateIdx = assem._Parent[targetLinkIndex].first;
				const Assembly::Mate& mate = assem._Mate[mateIdx];

				targetTrace.push_front(assem._Parent[targetLinkIndex]);

				targetLinkIndex = mate.getParentLinkIdx(assem._Parent[targetLinkIndex].second);
				targetLinkDepth--;
			}

			{
				unsigned int mateIdx = assem._Parent[referenceLinkIndex].first;
				const Assembly::Mate& mate = assem._Mate[mateIdx];

				referenceTrace.push_back(Assembly::reverseDirection(assem._Parent[referenceLinkIndex]));

				referenceLinkIndex = mate.getParentLinkIdx(assem._Parent[referenceLinkIndex].second);
				referenceLinkDepteh--;
			}
		}
		trace.splice(trace.end(), referenceTrace);
		trace.splice(trace.end(), targetTrace);

		for (list< pair< unsigned int, JointDirection >>::iterator iter = trace.begin(); iter != trace.end(); iter++)
		{
			unsigned int mateIdx = iter->first;

			if (assem._Mate[mateIdx]._joint->getDOF() != 0)
			{
				state.writeReturnMatrix(J,
					SE3::Ad(T) * assem.getJacobian(mateIdx, state.getJointStateByMateIndex(mateIdx), iter->second),
					0,
					state.getJointIndexByMateIndex(mateIdx),
					State::STATEJOINT);
			}
			T *= assem.getTransform(mateIdx, state.getJointStateByMateIndex(mateIdx), iter->second);
		}

		if (state.getDOF(State::PASSIVEJOINT) != 0)
		{
			if (assem._ClosedLoopConstraint.size() == 0)
				ReturnJ = J.block(0, 0, 6, state.getDOF(State::ACTIVEJOINT)).eval();
			else
			{
				MatrixX Jc = computeClosedLoopConstraintJacobian(assem, state, State::STATEJOINT);

				const MatrixX &Jca = Jc.block(0, 0, Jc.rows(), state.getDOF(State::ACTIVEJOINT));
				const MatrixX &Jcp = Jc.block(0, state.getDOF(State::ACTIVEJOINT), Jc.rows(), state.getDOF(State::PASSIVEJOINT));

				ReturnJ = J.block(0, 0, 6, state.getDOF(State::ACTIVEJOINT)) -
					J.block(0, state.getDOF(State::ACTIVEJOINT), 6, state.getDOF(State::PASSIVEJOINT)) * pInv(Jcp) * Jca;
			}
		}
		else
		{
			ReturnJ = J;
		}
		return pair< SE3, Matrix6X >(T, ReturnJ);
	}

	void Kinematics::solveInverseKinematics(const Assembly& assem, State& state, const SE3& goalT, const std::string& targetLinkMarkerName, const string& referenceLinkMarkerName)
	{
		unsigned int targetLinkIdx = assem.getLinkIndex(targetLinkMarkerName);
		unsigned int referenceLinkIdx;
		bool isTargetMarker = false;
		bool isReferenceMarker = false;

		if (targetLinkIdx == -1)
		{
			targetLinkIdx = assem.getLinkIndexByMarkerName(targetLinkMarkerName);
			utils::Log(targetLinkIdx == -1, "targetLinkMarkerName은 링크의 이름이거나 마커의 이름이어야 합니다.", true);
			isTargetMarker = true;
		}

		if (referenceLinkMarkerName.compare("") == 0)
		{
			referenceLinkIdx = assem._baseLink;
		}
		else
		{
			referenceLinkIdx = assem.getLinkIndex(referenceLinkMarkerName);
			if (referenceLinkIdx == -1)
			{
				referenceLinkIdx = assem.getLinkIndexByMarkerName(referenceLinkMarkerName);
				utils::Log(referenceLinkIdx == -1, "referenceLinkMarkerName은 링크의 이름이거나 마커의 이름이어야 합니다.", true);
				isReferenceMarker = true;
			}
		}

		if (isTargetMarker && isReferenceMarker)
		{
			return solveInverseKinematics(assem, state, (assem.getLinkPtr(referenceLinkIdx)->getMarker(referenceLinkMarkerName))*goalT*(assem.getLinkPtr(targetLinkIdx)->getMarker(targetLinkMarkerName)).inverse(), targetLinkIdx, referenceLinkIdx);
		}
		else if (isTargetMarker)
		{
			return solveInverseKinematics(assem, state, goalT*(assem.getLinkPtr(targetLinkIdx)->getMarker(targetLinkMarkerName)).inverse(), targetLinkIdx, referenceLinkIdx);
		}
		else if (isReferenceMarker)
		{
			return solveInverseKinematics(assem, state, (assem.getLinkPtr(referenceLinkIdx)->getMarker(referenceLinkMarkerName))*goalT, targetLinkIdx, referenceLinkIdx);
		}
		return solveInverseKinematics(assem, state, goalT, targetLinkIdx, referenceLinkIdx);
	}

	void Kinematics::solveInverseKinematics(const Assembly& assem, State& state, const SE3& goalT, const unsigned int targetLinkIndex, int referenceLinkIndex)
	{
		utils::Log(state.getDOF(State::ACTIVEJOINT) == 0, "Active Joint는 하나 이상이어야 합니다.", true);

		if (referenceLinkIndex == -1) referenceLinkIndex = assem._baseLink;
		VectorX S;
		pair< SE3, Matrix6X > TnJ;
		while (true)
		{
			TnJ = computeTransformNJacobian(assem, state, targetLinkIndex, referenceLinkIndex);
			if ((S = SE3::Log(goalT * TnJ.first.inverse())).norm() < InverseKinematicsExitCondition)
				break;
			state.addJointq(State::ACTIVEJOINT, pInv(TnJ.second) * S);
		}
	}

	void Kinematics::solveForwardKinematics(const SerialOpenChainAssembly& assem, State& state, const unsigned int options)
	{
		if (state.getJointReferenceFrame() != JointReferenceFrame::SPATIAL)
		{
			state.setInfoUpToDate(State::ALL_INFO, false);
			state.setJointReferenceFrame(JointReferenceFrame::SPATIAL);
		}

		if ((options & State::JOINTS_T_FROM_BASE | State::JOINTS_JACOBIAN | State::JOINTS_JACOBIAN_DOT | State::LINKS_POS | State::LINKS_VEL | State::LINKS_ACC) && !state.getInfoUpToDate(State::JOINTS_T_FROM_BASE))
		{
			SE3 T;
			state.getLinkState(assem._baseLink)._T = assem._socLink[assem._baseLink]._M;
			for (unsigned int i = 0; i < assem._Tree.size(); i++)
			{
				unsigned int mateIdx = assem._Tree[i].first;

				T *= assem.getTransform(mateIdx, state.getJointStateByMateIndex(mateIdx));
				state.getJointStateByMateIndex(mateIdx)._accumulatedT = T;
			}
			state.setInfoUpToDate(State::ALL_INFO, false);
			state.setInfoUpToDate(State::JOINTS_T_FROM_BASE, true);
		}

		if ((options & State::LINKS_POS) && !state.getInfoUpToDate(State::LINKS_POS))
		{
			state.getLinkState(assem._baseLink)._T = assem._socLink[assem._baseLink]._M;
			for (unsigned int i = 0; i < assem._Tree.size(); i++)
			{
				unsigned int mateIdx = assem._Tree[i].first;

				state.getLinkState(assem._Mate[mateIdx].getChildLinkIdx())._T = state.getJointStateByMateIndex(mateIdx)._accumulatedT * 
					assem._socLink[assem._Mate[mateIdx].getChildLinkIdx()]._M;
			}
			state.setInfoUpToDate(State::LINKS_POS);
		}

		if ((options & (State::JOINTS_JACOBIAN | State::JOINTS_JACOBIAN_DOT | State::LINKS_VEL | State::LINKS_ACC)) && !state.getInfoUpToDate(State::JOINTS_JACOBIAN))
		{
			for (unsigned int i = 0; i < assem._Tree.size(); i++)
			{
				unsigned int mateIdx = assem._Tree[i].first;
				if (i != 0)
				{
					state.getJointStateByMateIndex(mateIdx)._accumulatedJ = SE3::Ad(state.getJointStateByMateIndex(assem._Tree[i - 1].first)._accumulatedT) *
						assem.getJacobian(mateIdx, state.getJointStateByMateIndex(mateIdx));
				}
				else
				{
					state.getJointStateByMateIndex(mateIdx)._accumulatedJ = assem.getJacobian(mateIdx, state.getJointStateByMateIndex(mateIdx));
				}
			}
			state.setInfoUpToDate(State::JOINTS_JACOBIAN);
			state.setInfoUpToDate(State::JOINTS_JACOBIAN_DOT|State::LINKS_VEL|State::State::LINKS_ACC, false);
		}

		if ((options & State::LINKS_VEL) && !state.getInfoUpToDate(State::LINKS_VEL))
		{
			se3 V;
			V.setZero();

			state.getLinkState(assem._baseLink)._V = V;
			for (unsigned int i = 0; i < assem._Tree.size(); i++)
			{
				unsigned int mateIdx = assem._Tree[i].first;
				V += state.getJointStateByMateIndex(mateIdx)._accumulatedJ * state.getJointStateByMateIndex(mateIdx).getqdot();
				state.getLinkState(assem._Mate[mateIdx].getChildLinkIdx(assem._Tree[i].second))._V = V;
			}
			state.setInfoUpToDate(State::LINKS_VEL);
		}

		if ((options & (State::JOINTS_JACOBIAN_DOT | State::LINKS_ACC)) && !state.getInfoUpToDate(State::JOINTS_JACOBIAN_DOT))
		{
			Matrix6 adjoint;
			adjoint.setZero();

			for (unsigned int i = 0; i < assem._Tree.size(); i++)
			{
				unsigned int mateIdx = assem._Tree[i].first;

				if (i != 0)
				{
					state.getJointStateByMateIndex(mateIdx)._accumulatedJDot = adjoint * state.getJointStateByMateIndex(mateIdx)._accumulatedJ +
						assem.getJacobianDot(mateIdx, state.getJointStateByMateIndex(mateIdx));
				}
				else
				{
					state.getJointStateByMateIndex(mateIdx)._accumulatedJDot = assem.getJacobianDot(mateIdx, state.getJointStateByMateIndex(mateIdx));
				}
				for (int j = 0; j < state.getJointStateByMateIndex(mateIdx)._accumulatedJ.cols(); j++)
				{
					adjoint += SE3::ad(state.getJointStateByMateIndex(mateIdx)._accumulatedJ.col(j)) * state.getJointStateByMateIndex(mateIdx).getqdot()[j];
				}
			}
			state.setInfoUpToDate(State::JOINTS_JACOBIAN_DOT);
			state.setInfoUpToDate(State::LINKS_ACC,false);
		}

		if ((options & State::State::LINKS_ACC) && !state.getInfoUpToDate(State::LINKS_ACC))
		{
			se3 VDot;
			VDot.setZero();

			state.getLinkState(assem._baseLink)._VDot = VDot;
			for (unsigned int i = 0; i < assem._Tree.size(); i++)
			{
				unsigned int mateIdx = assem._Tree[i].first;

				VDot += state.getJointStateByMateIndex(mateIdx)._accumulatedJ * state.getJointStateByMateIndex(mateIdx).getqddot() +
					state.getJointStateByMateIndex(mateIdx)._accumulatedJDot * state.getJointStateByMateIndex(mateIdx).getqdot();
				state.getLinkState(assem._Mate[mateIdx].getChildLinkIdx(assem._Tree[i].second))._VDot = VDot;
			}
			state.setInfoUpToDate(State::LINKS_ACC);
		}
	}

	Math::SE3 Kinematics::calculateEndeffectorFrame(const SerialOpenChainAssembly& assem, State& state)
	{
		solveForwardKinematics(assem, state, State::JOINTS_T_FROM_BASE);
		return state.getJointStateByMateIndex(assem._Tree[assem._Tree.size() - 1].first)._accumulatedT * assem._socLink[assem._endeffectorLink]._M;
	}

	Matrix6X Kinematics::computeJacobian(const SerialOpenChainAssembly& assem, State& state)
	{
		solveForwardKinematics(assem, state, State::JOINTS_JACOBIAN);

		MatrixX J(6, state.getDOF(State::ACTIVEJOINT));

		for (unsigned int i = 0; i < assem._Mate.size(); i++)
		{
			unsigned int mateIdx = assem._Tree[i].first;

			state.writeReturnMatrix(J,
				state.getJointStateByMateIndex(mateIdx)._accumulatedJ,
				0,
				state.getJointIndexByMateIndex(mateIdx),
				State::ACTIVEJOINT);
		}

		return J;
	}

	Matrix6X Kinematics::computeJacobianDot(const SerialOpenChainAssembly& assem, State& state)
	{
		solveForwardKinematics(assem, state, State::JOINTS_JACOBIAN_DOT);

		MatrixX JDot(6, state.getDOF(State::ACTIVEJOINT));

		for (unsigned int i = 0; i < assem._Mate.size(); i++)
		{
			unsigned int mateIdx = assem._Tree[i].first;

			state.writeReturnMatrix(JDot,
				state.getJointStateByMateIndex(mateIdx)._accumulatedJDot,
				0,
				state.getJointIndexByMateIndex(mateIdx),
				State::ACTIVEJOINT);
		}

		return JDot;
	}

	void Kinematics::solveInverseKinematics(const SerialOpenChainAssembly& assem, State& state, const SE3 goalT)
	{
		VectorX S;
		Matrix6X J;
		while (true)
		{
			solveForwardKinematics(assem, state, State::JOINTS_T_FROM_BASE);

			J = computeJacobian(assem, state);
			if ((S = SE3::Log(goalT * (state.getJointStateByMateIndex(assem._Tree[assem._Tree.size() - 1].first)._accumulatedT * assem._socLink[assem._endeffectorLink]._M).inverse())).norm() < InverseKinematicsExitCondition)
				break;
			state.addJointq(State::ACTIVEJOINT, pInv(J) * S);
		}
	}
}