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
		vector< dse3, Eigen::aligned_allocator<dse3>> externalF;
		externalF.resize(assem.getLinkList().size());
		for (unsigned int i = 0; i < externalF.size(); i++)
		{
			externalF[i].setZero();
		}
		for (unsigned int i = 0; i < F.size(); i++)
		{
			externalF[F[i].first] += F[i].second;
		}

		Kinematics::solveForwardKinematics(assem, state, State::LINKS_VEL | State::LINKS_ACC);

		dse3 netF;
		se3 V;
		se3 VDot;
		Matrix6 Adjoint;
		netF.setZero();
		for (unsigned int i = 0; i < assem._Tree.size(); i++)
		{
			unsigned int mateIdx = assem._Tree[assem._Tree.size() - i - 1].first;
			unsigned int linkIdx = assem._Mate[assem._Tree[assem._Tree.size() - i - 1].first].getChildLinkIdx();

			Adjoint = SE3::InvAd(state.getJointStateByMateIndex(mateIdx)._accumulatedT);
			V = Adjoint * state.getLinkState(linkIdx)._V;
			VDot = Adjoint * state.getLinkState(linkIdx)._VDot;

			netF = SE3::Ad(state.getJointStateByMateIndex(mateIdx)._accumulatedT).transpose() * netF + externalF[linkIdx] +
				(Matrix6)assem._socLink[linkIdx]._G*VDot -
				SE3::adTranspose(V)*(Matrix6)assem._socLink[linkIdx]._G*V;
			netF = Adjoint.transpose() * netF;

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

		Kinematics::solveForwardKinematics(assem, state, State::LINKS_VEL | State::JOINTS_JACOBIAN | State::JOINTS_T_FROM_BASE);

		//
		//	Initialization
		//

		//	Define indices.
		unsigned int lastMateIdx = assem._Tree.size() - 1;
		unsigned int cLinkIdx = assem._Mate[assem._Tree[lastMateIdx].first].getChildLinkIdx();
		unsigned int pLinkIdx = assem._Mate[assem._Tree[lastMateIdx].first].getParentLinkIdx();
		Inertia linkInertia = assem._socLink[cLinkIdx]._G.getTransformed(state.getJointStateByMateIndex(assem._Tree[lastMateIdx].first)._accumulatedT, true);

		state.getLinkState(cLinkIdx)._Ja.noalias() = linkInertia;
		state.getLinkState(cLinkIdx)._b.noalias() =
			-SE3::adTranspose(state.getLinkState(cLinkIdx)._V)
			* (static_cast<Matrix6&>(linkInertia)
			* state.getLinkState(cLinkIdx)._V);
		if (!extF[lastMateIdx + 1].isZero())
			state.getLinkState(cLinkIdx)._b.noalias() -= SE3::InvAd(state.getLinkState(cLinkIdx)._T).transpose() * extF[lastMateIdx + 1];

		//	Intermediate variables for efficient computation.
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
			State::JointState&	jStat = state.getJointStateByMateIndex(assem._Tree[idx].first);

			//	Pre-calculate frequently used variables.
			linkInertia = assem._socLink[pLinkIdx]._G.getTransformed(state.getJointStateByMateIndex(assem._Tree[idx - 1].first)._accumulatedT, true);
			Ja_S = (Matrix6)state.getLinkState(cLinkIdx)._Ja * jStat._accumulatedJ;
			inv_ST_Ja_S = (jStat._accumulatedJ.transpose()*Ja_S).inverse();
			ad_V_S_qdot = SE3::ad(state.getLinkState(cLinkIdx)._V) * (jStat._accumulatedJ * jStat.getqdot());

			//	Articulated inertia of parent link of 'idx'-th mate.
			state.getLinkState(pLinkIdx)._Ja.noalias() =
				static_cast<Matrix6&>(linkInertia)
				+ state.getLinkState(cLinkIdx)._Ja
				- Ja_S * inv_ST_Ja_S * Ja_S.transpose();
			
			//	Bias force of parent link of 'idx'-th mate.
			state.getLinkState(pLinkIdx)._b.noalias() =
				-SE3::adTranspose(state.getLinkState(pLinkIdx)._V) * (static_cast<Matrix6&>(linkInertia) * state.getLinkState(pLinkIdx)._V)
				+ state.getLinkState(cLinkIdx)._b
				+ state.getLinkState(cLinkIdx)._Ja * ad_V_S_qdot
				+ Ja_S * inv_ST_Ja_S * (jStat._tau - Ja_S.transpose() * ad_V_S_qdot - jStat._accumulatedJ.transpose()*state.getLinkState(cLinkIdx)._b);
			
			//	Apply external force if exists.
			if (!extF[idx].isZero())
				state.getLinkState(pLinkIdx)._b.noalias() -= SE3::InvAd(state.getLinkState(pLinkIdx)._T).transpose() * extF[idx];
		}

		//
		//	Forward recursion
		//
		for (unsigned idx = 0; idx < assem._Tree.size(); idx++)
		{
			//	Define indices.
			cLinkIdx = assem._Mate[assem._Tree[idx].first].getChildLinkIdx();
			pLinkIdx = assem._Mate[assem._Tree[idx].first].getParentLinkIdx();
			State::JointState&	jStat = state.getJointStateByMateIndex(assem._Tree[idx].first);

			//	Pre-calculate frequently used variables.
			//	TODO: Avoid duplicated calculation. (Most of these need only a single calculation among back/forward iteration)
			Ja_S = static_cast<Matrix6&>(state.getLinkState(cLinkIdx)._Ja) * jStat._accumulatedJ;
			inv_ST_Ja_S = (jStat._accumulatedJ.transpose()*Ja_S).inverse();
			ad_V_S_qdot = SE3::ad(state.getLinkState(cLinkIdx)._V) * (jStat._accumulatedJ * jStat.getqdot());
			 
			VectorX qddot =
				inv_ST_Ja_S * (jStat._tau - Ja_S.transpose()*(state.getLinkState(pLinkIdx)._VDot + ad_V_S_qdot) - jStat._accumulatedJ.transpose()*state.getLinkState(cLinkIdx)._b);
			state.setJointqddot(assem._Tree[idx].first, qddot);

			state.getLinkState(cLinkIdx)._VDot = jStat._accumulatedJ * qddot + state.getLinkState(pLinkIdx)._VDot + ad_V_S_qdot;
		}

	}

	void Dynamics::solveForwardDynamics_renew(const Model::SerialOpenChainAssembly & assem, Model::State & state, const std::vector<std::pair<unsigned int, Math::dse3>>& extForce)
	{
		//	Term 'extF' includes the force applied on base link which will be ignored.
		vector< dse3, Eigen::aligned_allocator<dse3>> extF(assem.getLinkList().size());
		for (unsigned int i = 0; i < extF.size(); i++)
			extF[i].setZero();
		for (unsigned int i = 0; i < extForce.size(); i++)
			extF[extForce[i].first] += extForce[i].second;

		Kinematics::solveForwardKinematics(assem, state, State::LINKS_VEL | State::JOINTS_JACOBIAN | State::JOINTS_T_FROM_BASE);

		//
		//	Initialization
		//

		//	Define indices.
		unsigned int lastMateIdx = assem._Tree.size() - 1;
		unsigned int cLinkIdx = assem._Mate[assem._Tree[lastMateIdx].first].getChildLinkIdx();
		unsigned int pLinkIdx = assem._Mate[assem._Tree[lastMateIdx].first].getParentLinkIdx();
		Inertia linkInertia = assem._socLink[cLinkIdx]._G.getTransformed(state.getJointStateByMateIndex(assem._Tree[lastMateIdx].first)._accumulatedT, true);

		state.getLinkState(cLinkIdx)._Ja.noalias() = linkInertia;
		state.getLinkState(cLinkIdx)._b.noalias() =
			-SE3::adTranspose(state.getLinkState(cLinkIdx)._V)
			* (static_cast<Matrix6&>(linkInertia)
			   * state.getLinkState(cLinkIdx)._V);
		if (!extF[lastMateIdx + 1].isZero())
			state.getLinkState(cLinkIdx)._b.noalias() -= SE3::InvAd(state.getLinkState(cLinkIdx)._T).transpose() * extF[lastMateIdx + 1];

		//	Intermediate variables for efficient computation.
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
			State::JointState&	jStat = state.getJointStateByMateIndex(assem._Tree[idx].first);

			//	Pre-calculate frequently used variables.
			linkInertia = assem._socLink[pLinkIdx]._G.getTransformed(state.getJointStateByMateIndex(assem._Tree[idx - 1].first)._accumulatedT, true);
			Ja_S = (Matrix6)state.getLinkState(cLinkIdx)._Ja * jStat._accumulatedJ;
			inv_ST_Ja_S = (jStat._accumulatedJ.transpose()*Ja_S).inverse();
			ad_V_S_qdot = SE3::ad(state.getLinkState(cLinkIdx)._V) * (jStat._accumulatedJ * jStat.getqdot());

			//	Articulated inertia of parent link of 'idx'-th mate.
			state.getLinkState(pLinkIdx)._Ja.noalias() =
				static_cast<Matrix6&>(linkInertia)
				+ state.getLinkState(cLinkIdx)._Ja
				- Ja_S * inv_ST_Ja_S * Ja_S.transpose();

			//	Bias force of parent link of 'idx'-th mate.
			state.getLinkState(pLinkIdx)._b.noalias() =
				-SE3::adTranspose(state.getLinkState(pLinkIdx)._V) * (static_cast<Matrix6&>(linkInertia) * state.getLinkState(pLinkIdx)._V)
				+ state.getLinkState(cLinkIdx)._b
				+ state.getLinkState(cLinkIdx)._Ja * ad_V_S_qdot
				+ Ja_S * inv_ST_Ja_S * (jStat._tau - Ja_S.transpose() * ad_V_S_qdot - jStat._accumulatedJ.transpose()*state.getLinkState(cLinkIdx)._b);

			//	Apply external force if exists.
			if (!extF[idx].isZero())
				state.getLinkState(pLinkIdx)._b.noalias() -= SE3::InvAd(state.getLinkState(pLinkIdx)._T).transpose() * extF[idx];
		}

		//
		//	Forward recursion
		//
		for (unsigned idx = 0; idx < assem._Tree.size(); idx++)
		{
			//	Define indices.
			cLinkIdx = assem._Mate[assem._Tree[idx].first].getChildLinkIdx();
			pLinkIdx = assem._Mate[assem._Tree[idx].first].getParentLinkIdx();
			State::JointState&	jStat = state.getJointStateByMateIndex(assem._Tree[idx].first);

			//	Pre-calculate frequently used variables.
			//	TODO: Avoid duplicated calculation. (Most of these need only a single calculation among back/forward iteration)
			Ja_S = static_cast<Matrix6&>(state.getLinkState(cLinkIdx)._Ja) * jStat._accumulatedJ;
			inv_ST_Ja_S = (jStat._accumulatedJ.transpose()*Ja_S).inverse();
			ad_V_S_qdot = SE3::ad(state.getLinkState(cLinkIdx)._V) * (jStat._accumulatedJ * jStat.getqdot());

			VectorX qddot =
				inv_ST_Ja_S * (jStat._tau - Ja_S.transpose()*(state.getLinkState(pLinkIdx)._VDot + ad_V_S_qdot) - jStat._accumulatedJ.transpose()*state.getLinkState(cLinkIdx)._b);
			state.setJointqddot(assem._Tree[idx].first, qddot);

			state.getLinkState(cLinkIdx)._VDot = jStat._accumulatedJ * qddot + state.getLinkState(pLinkIdx)._VDot + ad_V_S_qdot;
		}
	}

}