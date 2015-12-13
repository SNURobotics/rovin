#include "Kinematics.h"

#include <cmath>

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

		if ((options & (State::LINKS_VEL | State::LINKS_ACC)) == (State::LINKS_VEL | State::LINKS_ACC) && (!state.getInfoUpToDate(State::LINKS_VEL) || !state.getInfoUpToDate(State::LINKS_ACC)))
		{
			se3 V;
			se3 VDot;
			se3 temp;
			V.setZero();
			VDot.setZero();
			VDot(5) = 9.8;

			state.getLinkState(assem._baseLink)._V = V;
			state.getLinkState(assem._baseLink)._VDot = VDot;
			for (unsigned int i = 0; i < assem._Tree.size(); i++)
			{
				unsigned int mateIdx = assem._Tree[i].first;
				state.getJointStateByMateIndex(mateIdx)._accumulatedJ.resize(6, state.getJointState(mateIdx).getDOF());
				for (unsigned int j = 0; j < state.getJointState(mateIdx).getDOF(); j++)
				{
					if (i == 0) temp = assem.getJacobian(mateIdx, state.getJointStateByMateIndex(mateIdx)).col(j);
					else temp = SE3::Ad(state.getJointStateByMateIndex(assem._Tree[i - 1].first)._accumulatedT, assem.getJacobian(mateIdx, state.getJointStateByMateIndex(mateIdx)).col(j));
					state.getJointStateByMateIndex(mateIdx)._accumulatedJ.col(j) = temp;
					VDot += temp*state.getJointStateByMateIndex(mateIdx).getqddot()(j) + SE3::ad(V, temp)*state.getJointStateByMateIndex(mateIdx).getqdot()(j);
					V += temp*state.getJointStateByMateIndex(mateIdx).getqdot()(j);
				}
				state.getLinkState(assem._Mate[mateIdx].getChildLinkIdx(assem._Tree[i].second))._V = V;
				state.getLinkState(assem._Mate[mateIdx].getChildLinkIdx(assem._Tree[i].second))._VDot = VDot;
			}
			state.setInfoUpToDate(State::JOINTS_JACOBIAN);
			state.setInfoUpToDate(State::LINKS_VEL);
			state.setInfoUpToDate(State::LINKS_ACC);
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

		if (((options & (State::JOINTS_JACOBIAN | State::JOINTS_JACOBIAN_DOT)) && !state.getInfoUpToDate(State::JOINTS_JACOBIAN)) ||
			((options & State::LINKS_VEL) && !state.getInfoUpToDate(State::LINKS_VEL)) ||
			((options & State::LINKS_ACC) && !state.getInfoUpToDate(State::LINKS_ACC)))
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

		if (((options & State::JOINTS_JACOBIAN_DOT) && !state.getInfoUpToDate(State::JOINTS_JACOBIAN_DOT)) ||
			((options & State::LINKS_ACC) && !state.getInfoUpToDate(State::LINKS_ACC)))
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
			VDot(5) = 9.8;
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

	vector<VectorX> Kinematics::solveInverseKinematicsOnlyForEfort(const Model::SerialOpenChainAssembly& assem, const Math::SE3& goalT)
	{
		MatrixX screw(6, 6);
		Vector3 pw;
		bool valid[2];
		valid[0] = false;
		valid[1] = false;

		for (int i = 0; i < 6; i++)
		{
			screw.col(i) = assem._socMate[i]._axes.col(0);
		}
		pw = assem._socLink[4]._M.getPosition();

		SE3 M = assem._socLink[6]._M;
		SE3 gd = goalT*M.inverse();
		Vector3 pw_now = gd.getRotation().matrix()*pw + gd.getPosition();
		Vector3 z1 = screw.col(0).head<3>();
		Vector3 p1 = z1.cross(screw.col(1).tail<3>());
		Vector3 pw_proj1 = pw - (z1.transpose()*(pw - p1))(0)*z1;
		Vector3 pw_now_proj1 = pw_now - (z1.transpose()*(pw_now - p1))(0)*z1;

		Vector4 q1;
		q1(0) = std::acos((pw_proj1.transpose()*pw_now_proj1)(0) / pw_proj1.norm() / pw_now_proj1.norm());
		if (((pw_proj1.cross(pw_now_proj1)).transpose()*z1)(0) < 0.0)
		{
			q1(0) = -q1(0);
		}
		q1(1) = q1(0);
		q1(2) = q1(0) + Math::PI;
		q1(3) = q1(2);

		Vector3 z2 = screw.col(1).head<3>();
		Vector3 p2 = screw.col(1).head<3>().cross(screw.col(1).tail<3>());
		Vector3 p3 = screw.col(2).head<3>().cross(screw.col(2).tail<3>());

		Vector3 pw_proj2 = pw - (z2.transpose()*(pw - p2))(0)*z2;
		Vector3 p3_proj2 = p3 - (z2.transpose()*(p3 - p2))(0)*z2;
		Real L2 = (p3_proj2 - p2).norm();
		Vector3 vec3w = pw_proj2 - p3_proj2;
		Real L3 = vec3w.norm();
		Real alpha = atan2(vec3w(2), vec3w(0));

		Vector4 q2, q3;

		Vector2 theta3;
		Vector2 theta2;
		for (int i = 0; i < 2; i++)
		{
			SE3 T1 = SE3::Exp(screw.col(0).head<3>() * q1(2 * i), screw.col(0).tail<3>() * q1(2 * i));
			Vector3 z2_now = T1.getRotation().matrix() * screw.col(1).head<3>();
			Vector3 p2_now = T1.getRotation().matrix() * p2 + T1.getPosition();
			Vector3 pw_now_proj2_now = pw_now - (z2_now.transpose()*(pw_now - p2_now))(0)*z2_now;
			Vector3 vec2w = pw_now_proj2_now - p2_now;
			Real L = vec2w.norm();
			if (L > L2 + L3)
				continue;
			valid[i] = true;
			theta3(0) = std::acos(0.5*(L2*L2 + L3*L3 - L*L) / (L2*L3));
			theta3(1) = -theta3(0);
			Real phi = std::acos(0.5*(L2*L2 + L*L - L3*L3) / (L2*L));
			Vector3 vec_y = (vec2w.transpose()*screw.col(0).head<3>())(0) * screw.col(0).head<3>();
			Vector3 vec_x = vec2w - vec_y;
			Real theta = std::atan2(screw.col(0).head<3>().transpose()*(vec_y), vec_x.norm());
			theta2(0) = theta + phi;
			theta2(1) = theta - phi;

			if((z1.transpose()*vec_x.cross(z2_now))(0) > 0.0)
			{
				theta3 = theta3 - alpha*Vector2::Ones();
				q2.segment(i * 2, 2) = 0.5*Math::PI*Vector2::Ones() - theta2;
				q3.segment(i * 2, 2) = 0.5*Math::PI*Vector2::Ones() - theta3;
			}
			else
			{
				theta3 = theta3 + alpha*Vector2::Ones();
				q2.segment(i * 2, 2) = theta2 - 0.5*Math::PI*Vector2::Ones();
				q3.segment(i * 2, 2) = 0.5*Math::PI*Vector2::Ones() + theta3;
			}
		}

		Eigen::Matrix<Math::Real, 8, 1> q4, q5, q6;
		for (int i = 0; i < 4; i++)
		{
			SE3 g = (SE3::Exp(screw.col(0).head<3>()*q1(i), screw.col(0).tail<3>()*q1(i)) *
				SE3::Exp(screw.col(1).head<3>()*q2(i), screw.col(1).tail<3>()*q2(i)) *
				SE3::Exp(screw.col(2).head<3>()*q3(i), screw.col(2).tail<3>()*q3(i))).inverse() * gd;

			Vector3 z = g.getRotation().matrix()*screw.col(5).head<3>();
			Real theta5 = std::acos((screw.col(5).head<3>().transpose()*z)(0));
			Vector3 z_proj = z - (z.transpose()*screw.col(5).head<3>())(0) * screw.col(5).head<3>();
			Vector3 vec = screw.col(4).head<3>().cross(screw.col(3).head<3>());
			Real n_z_proj = z_proj.norm();
			Real theta4;
			if (n_z_proj < RealEps)
			{
				theta4 = 0;
			}
			else
			{
				theta4 = std::acos((vec.transpose() * z_proj)(0) / z_proj.norm());
				if ((screw.col(3).head<3>().transpose() * vec.cross(z_proj))(0) > 0.0)
				{
					theta4 = abs(theta4);
				}
				else
				{
					theta4 = -abs(theta4);
				}
			}

			SE3 g_ = (SE3::Exp(screw.col(3).head<3>()*theta4, screw.col(3).tail<3>()*theta4) *
				SE3::Exp(screw.col(4).head<3>()*theta5, screw.col(4).tail<3>()*theta5)).inverse() * g;
			Vector3 w6 = SE3::Log(g_).head<3>();
			Real theta6 = w6.norm();
			if ((screw.col(5).head<3>().transpose() * w6)(0) < 0.0)
			{
				theta6 = -theta6;
			}
			if (theta4 < 0.0)
			{
				q4.segment(2 * i, 2) << theta4, theta4 + Math::PI;
			}
			else
			{
				q4.segment(2 * i, 2) << theta4, theta4 - Math::PI;
			}
			q5.segment(2 * i, 2) << theta5, -theta5;
			if (theta6 < 0.0)
			{
				q6.segment(2 * i, 2) << theta6, theta6 + Math::PI;
			}
			else
			{
				q6.segment(2 * i, 2) << theta6, theta6 - Math::PI;
			}
		}
		Eigen::Matrix<Math::Real, 8, 1> newq1, newq2, newq3;
		newq1(0) = newq1(1) = q1(0);
		newq1(2) = newq1(3) = q1(1);
		newq1(4) = newq1(5) = q1(2);
		newq1(6) = newq1(7) = q1(3);

		newq2(0) = newq2(1) = q2(0);
		newq2(2) = newq2(3) = q2(1);
		newq2(4) = newq2(5) = q2(2);
		newq2(6) = newq2(7) = q2(3);

		newq3(0) = newq3(1) = q3(0);
		newq3(2) = newq3(3) = q3(1);
		newq3(4) = newq3(5) = q3(2);
		newq3(6) = newq3(7) = q3(3);

		std::vector<VectorX> q;
		VectorX qelem(6);
		for (int i = 0; i < 8; i++)
		{
			if (!valid[0] && i >= 0 && i < 4) continue;
			if (!valid[1] && i >= 4 && i < 8) continue;

			qelem(0) = newq1(i);
			qelem(1) = newq2(i);
			qelem(2) = newq3(i);
			qelem(3) = q4(i);
			qelem(4) = q5(i);
			qelem(5) = q6(i);
			q.push_back(qelem);
		}

		return q;
	}
}