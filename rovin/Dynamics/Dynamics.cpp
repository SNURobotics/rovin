#include "Dynamics.h"

using namespace std;
using namespace rovin::Math;
using namespace rovin::Model;

namespace rovin
{
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
		vector< dse3 > externalF;
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
}