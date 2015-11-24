#include "Kinematics.h"

#include <rovin/utils/Diagnostic.h>
#include <rovin/Math/Constant.h>
#include <rovin/Math/LinearAlgebra.h>

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

	MatrixX Kinematics::computeClosedLoopConstraintJacobian(const Assembly& assem, State& state, const State::RETURN_STATE& return_state)
	{
		if (assem._ClosedLoopConstraint.size() == 0)
			return MatrixX::Zero(6, state.returnDof(return_state));

		MatrixX J(assem._ClosedLoopConstraint.size() * 6, state.returnDof(return_state));
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
			state.addPassiveJointq(-pInv(Kinematics::computeClosedLoopConstraintJacobian(assem, state, State::PASSIVEJOINT))*S);
		}
	}

	void Kinematics::solveForwardKinematics(const Model::Assembly & assem, Model::State & state)
	{
		solveClosedLoopConstraint(assem, state);

		for (unsigned int i = 0; i < assem._Tree.size(); i++)
		{
			unsigned int mateIdx = assem._Tree[i].first;

			//	update link state by propagation
			state.getLinkState(assem._Mate[mateIdx].getChildLinkIdx(assem._Tree[i].second))._T =
				state.getLinkState(assem._Mate[mateIdx].getParentLinkIdx(assem._Tree[i].second))._T *
				assem.getTransform(mateIdx, state.getJointStateByMateIndex(mateIdx), assem._Tree[i].second);
		}
	}

	Matrix6X Kinematics::computeJacobian(const Model::Assembly& assem, Model::State& state, const std::string& targetLinkMarkerName, const std::string& referenceLinkMarkerName)
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

	Matrix6X Kinematics::computeJacobian(const Model::Assembly& assem, Model::State& state, unsigned int targetLinkIndex, int referenceLinkIndex)
	{
		return computeTransformNJacobian(assem, state, targetLinkIndex, referenceLinkIndex).second;
	}

	pair< SE3, Matrix6X > Kinematics::computeTransformNJacobian(const Model::Assembly& assem, Model::State& state, unsigned int targetLinkIndex, int referenceLinkIndex)
	{
		solveClosedLoopConstraint(assem, state);

		utils::Log(state.getActiveJointDof() == 0, "Active Joint는 하나 이상이어야 합니다.", true);
		MatrixX J(6, state.returnDof(State::STATEJOINT));
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

		if (state.getTotalJointDof() != state.getActiveJointDof())
		{
			if (assem._ClosedLoopConstraint.size() == 0)
				ReturnJ = J.block(0, 0, 6, state.getActiveJointDof()).eval();
			else
			{
				MatrixX Jc = computeClosedLoopConstraintJacobian(assem, state, State::STATEJOINT);

				const MatrixX &Jca = Jc.block(0, 0, Jc.rows(), state.getActiveJointDof());
				const MatrixX &Jcp = Jc.block(0, state.getActiveJointDof(), Jc.rows(), state.getTotalJointDof() - state.getActiveJointDof());

				ReturnJ = J.block(0, 0, 6, state.getActiveJointDof()) -
					J.block(0, state.getActiveJointDof(), 6, state.getTotalJointDof() - state.getActiveJointDof()) * pInv(Jcp) * Jca;
			}
		}
		else
		{
			ReturnJ = J;
		}
		return pair< SE3, Matrix6X >(T, ReturnJ);
	}

	void Kinematics::solveInverseKinematics(const Model::Assembly& assem, Model::State& state, const Math::SE3& goalT, const std::string& targetLinkMarkerName, const std::string& referenceLinkMarkerName)
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

	void Kinematics::solveInverseKinematics(const Model::Assembly& assem, Model::State& state, const Math::SE3& goalT, const unsigned int targetLinkIndex, int referenceLinkIndex)
	{
		utils::Log(state.getActiveJointDof() == 0, "Active Joint는 하나 이상이어야 합니다.", true);

		if (referenceLinkIndex == -1) referenceLinkIndex = assem._baseLink;
		VectorX S;
		pair< SE3, Matrix6X > TnJ;
		while (true)
		{
			TnJ = computeTransformNJacobian(assem, state, targetLinkIndex, referenceLinkIndex);
			if ((S = SE3::Log(goalT * TnJ.first.inverse())).squaredNorm() < RealEps)
				break;
			state.addActiveJointq(pInv(TnJ.second) * S);
		}
	}

	void Kinematics::solveForwardKinematics(const Model::SerialOpenChainAssembly& assem, Model::State& state)
	{
		SE3 T;
		state.getLinkState(assem._baseLink)._T = assem._socLink[assem._baseLink]._M;
		for (unsigned int i = 0; i < assem._Tree.size(); i++)
		{
			unsigned int mateIdx = assem._Tree[i].first;

			T *= assem.getTransform(mateIdx, state.getJointStateByMateIndex(mateIdx));
			state.getLinkState(assem._Mate[mateIdx].getChildLinkIdx(assem._Tree[i].second))._T = T * assem._socLink[assem._Mate[mateIdx].getChildLinkIdx(assem._Tree[i].second)]._M;
		}
	}

	Math::SE3 Kinematics::calculateEndeffectorFrame(const Model::SerialOpenChainAssembly& assem, Model::State& state)
	{
		SE3 T;
		for (unsigned int i = 0; i < assem._Tree.size(); i++)
		{
			unsigned int mateIdx = assem._Tree[i].first;
			T *= assem.getTransform(mateIdx, state.getJointStateByMateIndex(mateIdx));
		}
		return T * assem._socLink[assem._endeffectorLink]._M;
	}
}