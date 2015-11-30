#include "Dynamics.h"

using namespace std;
using namespace rovin::Math;
using namespace rovin::Model;

namespace rovin
{
	void Dynamics::solveInverseDynamics(const Model::SerialOpenChainAssembly & assem, Model::State & state)
	{
		solveInverseDynamics(assem, state, std::vector< std::pair< unsigned int, Math::dse3 >>());
	}
	void Dynamics::solveInverseDynamics(const SerialOpenChainAssembly& assem, State& state, const vector< pair< string, dse3 >>& F)
	{
		vector< pair< unsigned int, dse3 >> externalF;

		for (unsigned int i = 0; i < F.size(); i++)
		{
			externalF.push_back(pair< unsigned int, dse3 >(state.getJointIndex(F[i].first), F[i].second));
		}
		solveInverseDynamics(assem, state, externalF);
	}

	void Dynamics::solveInverseDynamics(const SerialOpenChainAssembly& assem, State& state, const vector< pair< unsigned int, dse3 >>& F)
	{
		vector< dse3 , Eigen::aligned_allocator<dse3>> externalF;
		externalF.resize(assem.getLinkList().size());
		for (unsigned int i = 0; i < externalF.size(); i++)
		{
			externalF[i].setZero();
		}
		for (unsigned int i = 0; i < F.size(); i++)
		{
			externalF[F[i].first] += F[i].second;
		}

		Kinematics::solveForwardKinematics(assem, state, Kinematics::VELOCITY | Kinematics::ACCELERATION);

		dse3 netF;
		netF.setZero();
		for (unsigned int i = 0; i < assem._Tree.size(); i++)
		{
			unsigned int mateIdx = assem._Tree[assem._Tree.size() - i - 1].first;
			unsigned int linkIdx = assem._Mate[assem._Tree[assem._Tree.size() - i - 1].first].getChildLinkIdx();

			netF = netF + externalF[linkIdx] -
				(Matrix6)assem._socLink[linkIdx]._G*state.getLinkState(linkIdx)._VDot -
				SE3::adTranspose(state.getLinkState(linkIdx)._V)*(Matrix6)assem._socLink[linkIdx]._G*state.getLinkState(linkIdx)._V;

			state.getJointStateByMateIndex(mateIdx)._constraintF = netF;
			state.getJointStateByMateIndex(mateIdx)._tau = netF.transpose()*state.getJointStateByMateIndex(mateIdx)._accumulatedJ;
		}
	}

	void Dynamics::solveForwardDynamics(const Model::SerialOpenChainAssembly & assem, Model::State & state, const std::vector<std::pair<unsigned int, Math::dse3>>& extForce)
	{
		//	Term 'extF' includes the force applied on base link which will be ignored.
		vector< dse3, Eigen::aligned_allocator<dse3>> extF(assem.getLinkList().size());
		for (unsigned int i = 0; i < extF.size(); i++)
			extF[i].setZero();
		for (unsigned int i = 0; i < extForce.size(); i++)
			extF[extForce[i].first] += extForce[i].second;

		Kinematics::solveForwardKinematics(assem, state, Kinematics::VELOCITY);

		//
		//	Initialization
		//

		unsigned int lastMateIdx = assem._Tree.size() - 1;
		unsigned int cLinkIdx = assem._Mate[assem._Tree[lastMateIdx].first].getChildLinkIdx();
		unsigned int pLinkIdx = assem._Mate[assem._Tree[lastMateIdx].first].getParentLinkIdx();
		vector<Matrix6, Eigen::aligned_allocator<Matrix6>>	J_a(assem._Tree.size());
		vector<dse3, Eigen::aligned_allocator<dse3>>	bias(assem._Tree.size());

		J_a[lastMateIdx] = assem._socLink[cLinkIdx]._G;
		bias[lastMateIdx] = -SE3::adTranspose(state.getLinkState(cLinkIdx)._V) * assem._socLink[cLinkIdx]._G*state.getLinkState(cLinkIdx)._V;
		if (!extF[lastMateIdx + 1].isZero())
			bias[lastMateIdx] -= SE3::InvAd(state.getLinkState(cLinkIdx)._T).transpose() * extF[lastMateIdx + 1];

		//	Intermediate variables to efficient computation.
		Matrix6X Ja_S;
		MatrixX inv_ST_Ja_S;
		se3 ad_V_S_qdot;


		//
		//	Backward recursion
		//
		for (unsigned idx = lastMateIdx; idx > 0; idx--)
		{
			//	Define indices.
			cLinkIdx = assem._Mate[assem._Tree[idx].first].getChildLinkIdx();
			pLinkIdx = assem._Mate[assem._Tree[idx].first].getParentLinkIdx();
			State::JointState&	jStat = state.getJointStateByMateIndex(idx);

			//	Pre-calculate frequently used variables.
			Ja_S = (Matrix6)J_a[idx] * assem._socMate[assem._Tree[idx].first]._axes;
			inv_ST_Ja_S = (assem._socMate[assem._Tree[idx].first]._axes.transpose()*Ja_S).inverse();
			ad_V_S_qdot = SE3::ad(state.getLinkState(cLinkIdx)._V) * (assem._socMate[assem._Tree[idx].first]._axes * jStat.getqdot());

			//	Articulated inertia of parent link of 'idx'-th mate.

			//DEBUG//
			//cout << endl << assem._socLink[pLinkIdx]._G << endl << endl;
			//cout << (Matrix6)assem._socLink[pLinkIdx]._G
			//	+ J_a[idx]
			//	- Ja_S * inv_ST_Ja_S * Ja_S.transpose() << endl;
			//DEBUG//

			J_a[idx - 1] =
				(Matrix6)assem._socLink[pLinkIdx]._G
				+ J_a[idx]
				- Ja_S * inv_ST_Ja_S * Ja_S.transpose();
			//	Bias force of parent link of 'idx'-th mate.
			bias[idx - 1] =
				-SE3::adTranspose(state.getLinkState(pLinkIdx)._V) * assem._socLink[pLinkIdx]._G*state.getLinkState(pLinkIdx)._V
				+ bias[idx]
				+ J_a[idx] * ad_V_S_qdot
				+ Ja_S * inv_ST_Ja_S * (jStat._tau - Ja_S.transpose() * ad_V_S_qdot - assem._socMate[assem._Tree[idx].first]._axes.transpose()*bias[idx]);
			//	Apply external force if exists.
			if (!extF[idx].isZero())
				bias[idx - 1] -= SE3::InvAd(state.getLinkState(pLinkIdx)._T).transpose() * extF[idx];
		}

		//
		//	Forward recursion
		//
		for (unsigned idx = 0; idx < assem._Tree.size(); idx++)
		{
			//	Define indices.
			cLinkIdx = assem._Mate[assem._Tree[idx].first].getChildLinkIdx();
			pLinkIdx = assem._Mate[assem._Tree[idx].first].getParentLinkIdx();
			State::JointState&	jStat = state.getJointStateByMateIndex(idx);

			//	Pre-calculate frequently used variables.
			//	TODO: Avoid duplicated calculation. (Most of these need only a single calculation among back/forward iteration)
			Ja_S = (Matrix6)J_a[idx] * assem._socMate[assem._Tree[idx].first]._axes;
			inv_ST_Ja_S = (assem._socMate[assem._Tree[idx].first]._axes.transpose()*Ja_S).inverse();
			ad_V_S_qdot = SE3::ad(state.getLinkState(cLinkIdx)._V) * (assem._socMate[assem._Tree[idx].first]._axes * jStat.getqdot());

			VectorX qddot =
				inv_ST_Ja_S * (jStat._tau - Ja_S.transpose()*(state.getLinkState(pLinkIdx)._VDot + ad_V_S_qdot) - assem._socMate[assem._Tree[idx].first]._axes.transpose()*bias[idx]);
			state.setJointqddot(assem._Tree[idx].first, qddot);

			state.getLinkState(cLinkIdx)._VDot = assem._socMate[assem._Tree[idx].first]._axes * qddot + state.getLinkState(pLinkIdx)._VDot + ad_V_S_qdot;
		}

	}

}