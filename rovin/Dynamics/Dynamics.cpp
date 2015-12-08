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
		Matrix6 Adjoint;
		netF.setZero();
		for (unsigned int i = 0; i < assem._Tree.size(); i++)
		{
			unsigned int mateIdx = assem._Tree[assem._Tree.size() - i - 1].first;
			unsigned int linkIdx = assem._Mate[assem._Tree[assem._Tree.size() - i - 1].first].getChildLinkIdx();

			Adjoint = SE3::InvAd(state.getJointStateByMateIndex(mateIdx)._accumulatedT);

			netF = netF + externalF[linkIdx] +
				Adjoint.transpose()*((Matrix6)assem._socLink[linkIdx]._G*(Adjoint * state.getLinkState(linkIdx)._VDot)) -
				SE3::adTranspose(state.getLinkState(linkIdx)._V)*(Adjoint.transpose()*((Matrix6)assem._socLink[linkIdx]._G*(Adjoint * state.getLinkState(linkIdx)._V)));

			state.getJointStateByMateIndex(mateIdx)._constraintF = netF;
			state.getJointStateByMateIndex(mateIdx)._tau = netF.transpose()*state.getJointStateByMateIndex(mateIdx)._accumulatedJ;
			/*	+ assem.getJointPtrByMateIndex(mateIdx)->getConstDamper().cwiseProduct(state.getJointStateByMateIndex(mateIdx).getqdot());
			for (unsigned int j = 0; j < assem.getJointPtrByMateIndex(mateIdx)->getDOF(); j++)
			{
				if ( RealBigger(state.getJointStateByMateIndex(mateIdx).getqdot()(j), 0) )
					state.getJointStateByMateIndex(mateIdx)._tau(j) += assem.getJointPtrByMateIndex(mateIdx)->getConstFriction()(j);
				else if (RealLess(state.getJointStateByMateIndex(mateIdx).getqdot()(j), 0))
					state.getJointStateByMateIndex(mateIdx)._tau(j) -= assem.getJointPtrByMateIndex(mateIdx)->getConstFriction()(j);
			}*/
				
		}
	}

	pair<MatrixX, vector<MatrixX>> Dynamics::differentiateInverseDynamics(const Model::SerialOpenChainAssembly & assem, Model::State & state,
		const Math::MatrixX& dqdp, const Math::MatrixX& dqdotdp, const Math::MatrixX& dqddotdp,
		const std::vector<Math::MatrixX>& d2qdp2, ///< pN matrices of d/dk(dq/dp) (k = 1, ..., pN)
		const std::vector<Math::MatrixX>& d2qdotdp2, ///< pN matrices of d/dk(dqdot/dp) (k = 1, ..., pN) 
		const std::vector<Math::MatrixX>& d2qddotdp2, ///< pN matrices of d/dk(dqddot/dp) (k = 1, ..., pN) 
		const std::vector<std::pair<unsigned int, Math::dse3>>& Fext, ///< linkN spatial forces Fi (i = 1, ..., linkN)
		const std::vector<std::pair<unsigned int, Math::MatrixX>>& dFextdp, ///< linkN matrices of dFi/dp (i = 1, ..., linkN)
		const std::vector<std::pair<unsigned int, std::vector< Math::MatrixX >>>& d2Fextdp2, ///< linkN matrices of (pN matrices of d/dpk(dFi/dp), k = 1, ..., pN), (i = 1, ..., linkN)
		const bool needInverseDynamics)
	{
		if (needInverseDynamics)
		{
			solveInverseDynamics(assem, state, Fext);
		}

		Kinematics::solveForwardKinematics(assem, state, State::LINKS_VEL | State::LINKS_ACC);

		int qN = dqdp.rows();
		int pN = dqdp.cols();
		int linkN = assem.getLinkList().size();
		int DOF = state.getDOF(State::TARGET_JOINT::ASSEMJOINT);

		bool extForceExist = false;
		if (dFextdp.size() != 0 || d2Fextdp2.size() != 0) extForceExist = true;

		
		// variables for forward iteration
		vector< se3, Eigen::aligned_allocator<se3> > Vb(linkN), Vbdot(linkN);
		vector< MatrixX > dVbdp(linkN), dVbdotdp(linkN);
		vector< vector< MatrixX >> d2Vbdp2(linkN), d2Vbdotdp2(linkN);

		Matrix6 invAdeSiqi;
		se3 Si;
		Matrix6 adSi;
		se3 currVb, currVbdot;
		se3 pastVbdot;
		MatrixX currdVbdp, currdVbdotdp;
		MatrixX pastdVbdp, pastdVbdotdp;
		vector< MatrixX > currd2Vbdp2, currd2Vbdotdp2;


		// variables for backward iteration
		SE3 Tacci;
		Matrix6 AdTransTacci;
		Matrix6 invAdTranseSi1qi1;
		se3 Si1;
		Matrix6 adTransSi1;

		MatrixX dtaudp(DOF, pN);
		dtaudp.setZero();
		vector<MatrixX> d2taudp2(DOF, MatrixX::Zero(pN, pN));

		vector<MatrixX> dFbdp(linkN);
		vector<vector<MatrixX>> d2Fbdp2(linkN);

		dse3 currFb;
		dse3 pastFb;
		MatrixX currdFbdp;
		MatrixX pastdFbdp;
		vector<MatrixX> currd2Fbdp2;


		// forward iteration
		Vb[assem._baseLink] = currVb = state.getLinkState(assem._baseLink)._V;
		Vbdot[assem._baseLink] = currVbdot = state.getLinkState(assem._baseLink)._VDot;
		dVbdp[assem._baseLink] = currdVbdp = MatrixX::Zero(6, pN);
		dVbdotdp[assem._baseLink] = currdVbdotdp = MatrixX::Zero(6, pN);
		d2Vbdp2[assem._baseLink] = currd2Vbdp2 = vector< MatrixX >(pN, MatrixX::Zero(6, pN));
		d2Vbdotdp2[assem._baseLink] = currd2Vbdotdp2 = vector< MatrixX >(pN, MatrixX::Zero(6, pN));
		for (unsigned int i = 0; i < assem._Tree.size(); i++)
		{
			unsigned int mateID = assem._Tree[i].first;
			unsigned int plinkID = assem._Mate[mateID].getParentLinkIdx();
			unsigned int clinkID = assem._Mate[mateID].getChildLinkIdx();
			unsigned int dofIdx;

			currVb = Vb[plinkID];
			currVbdot = Vbdot[plinkID];
			currdVbdp = dVbdp[plinkID];
			currdVbdotdp = dVbdotdp[plinkID];
			currd2Vbdp2 = d2Vbdp2[plinkID];
			currd2Vbdotdp2 = d2Vbdotdp2[plinkID];

			for (unsigned int j = 0; j < state.getJointStateByMateIndex(mateID)._dof; j++)
			{
				if (j == 0) invAdeSiqi = SE3::Ad(state.getJointStateByMateIndex(mateID)._T[0].inverse());
				else invAdeSiqi = SE3::Ad(state.getJointStateByMateIndex(mateID)._T[j].inverse() * state.getJointStateByMateIndex(mateID)._T[j - 1]);

				pastVbdot = currVbdot;
				pastdVbdp = currdVbdp;
				pastdVbdotdp = currdVbdotdp;

				dofIdx = state.getAssemIndex(mateID) + j;
				Si = assem._socMate[mateID]._axes.col(j);
				adSi = SE3::ad(Si);
				currVb = invAdeSiqi*currVb + Si*state.getJointState(mateID).getqdot(j);
				currVbdot = invAdeSiqi*currVbdot + Si*state.getJointState(mateID).getqddot(j) +
					SE3::ad(currVb)*Si*state.getJointState(mateID).getqdot(j);

				currdVbdp = invAdeSiqi*currdVbdp + Si*dqdotdp.row(dofIdx) -
					(adSi*currVb)*dqdp.row(dofIdx);
				currdVbdotdp = invAdeSiqi*currdVbdotdp + Si*dqddotdp.row(dofIdx) -
					(adSi*(invAdeSiqi*pastVbdot))*dqdp.row(dofIdx) -
					adSi*currdVbdp*state.getJointState(mateID).getqdot(j) -
					(adSi*currVb)*dqdotdp.row(dofIdx);

				for (int k = 0; k < pN; k++)
				{
					currd2Vbdp2[k] = invAdeSiqi*currd2Vbdp2[k]  - 
						adSi*invAdeSiqi*pastdVbdp*dqdp(dofIdx, k) - 
						(adSi*currdVbdp.col(k))*dqdp.row(dofIdx);
					if (d2qdp2.size() != 0)
					{
						currd2Vbdp2[k] -= (adSi*currVb)*d2qdp2[k].row(dofIdx);
					}
					if (d2qdotdp2.size() != 0)
					{
						currd2Vbdp2[k] += Si*d2qdotdp2[k].row(dofIdx);
					}
					currd2Vbdotdp2[k] = invAdeSiqi*currd2Vbdotdp2[k]  -
						adSi*(
							invAdeSiqi*pastdVbdotdp*dqdp(dofIdx, k) +
							currd2Vbdp2[k] * state.getJointState(mateID).getqdot(j) +
							currdVbdp*dqdotdp(dofIdx, k)
							) -
						(adSi*(invAdeSiqi*pastdVbdotdp.col(k)))*dqdp.row(dofIdx) -
						(adSi*currdVbdp.col(k))*dqdotdp.row(dofIdx) +
						(adSi*(adSi*(invAdeSiqi*pastVbdot)))*dqdp.row(dofIdx)*dqdp(dofIdx, k);
					if (d2qdp2.size() != 0)
					{
						currd2Vbdotdp2[k] -= (adSi*(invAdeSiqi*pastVbdot))*d2qdp2[k].row(dofIdx);
					}
					if (d2qdotdp2.size() != 0)
					{
						currd2Vbdotdp2[k] -= (adSi*currVb)*d2qdotdp2[k].row(dofIdx);
					}
					if (d2qddotdp2.size() != 0)
					{
						currd2Vbdotdp2[k] += Si*d2qddotdp2[k].row(dofIdx);
					}
				}
				
			}

			Vb[clinkID] = currVb;
			Vbdot[clinkID] = currVbdot;
			dVbdp[clinkID] = currdVbdp;
			dVbdotdp[clinkID] = currdVbdotdp;
			d2Vbdp2[clinkID] = currd2Vbdp2;
			d2Vbdotdp2[clinkID] = currd2Vbdotdp2;
		}

		// backward iteration
		vector< MatrixX, Eigen::aligned_allocator<MatrixX>> extForce(assem.getLinkList().size());
		for (unsigned int i = 0; i < extForce.size(); i++)
			extForce[i] = MatrixX::Zero(6, 1);
		for (unsigned int i = 0; i < Fext.size(); i++)
			extForce[Fext[i].first] += Fext[i].second;

		vector< MatrixX, Eigen::aligned_allocator<MatrixX>> dextForcedp(assem.getLinkList().size());
		for (unsigned int i = 0; i < dextForcedp.size(); i++)
			dextForcedp[i] = MatrixX::Zero(6, pN);
		for (unsigned int i = 0; i < dFextdp.size(); i++)
			dextForcedp[dFextdp[i].first] += dFextdp[i].second;

		vector< vector< MatrixX >, Eigen::aligned_allocator< vector< MatrixX >>> d2extForcedp2(assem.getLinkList().size());
		for (unsigned int i = 0; i < d2extForcedp2.size(); i++)
			d2extForcedp2[i] = vector< MatrixX >(pN, MatrixX::Zero(6, pN));
		for (unsigned int i = 0; i < d2Fextdp2.size(); i++)
			for (int j = 0; j < pN; j++)
				d2extForcedp2[d2Fextdp2[i].first][j] += d2Fextdp2[i].second[j];

		

		// add external force
		if (extForceExist)
		{
			vector< vector < Matrix6, Eigen::aligned_allocator< Matrix6 >>> adJTransdqdpk(linkN, vector< Matrix6, Eigen::aligned_allocator< Matrix6 > >(pN, Matrix6::Zero()));
			vector< vector< vector < Matrix6, Eigen::aligned_allocator< Matrix6 >> >> adadJJdqdpdqkdpl(linkN, vector< vector< Matrix6, Eigen::aligned_allocator< Matrix6 > >>(pN, vector< Matrix6, Eigen::aligned_allocator< Matrix6 > >(pN, Matrix6::Zero())));
			vector< vector< vector < Matrix6, Eigen::aligned_allocator< Matrix6 >> >> adJTransd2qdpkdpl(linkN, vector< vector< Matrix6, Eigen::aligned_allocator< Matrix6 > >>(pN, vector< Matrix6, Eigen::aligned_allocator< Matrix6 > >(pN, Matrix6::Zero())));
			
			for (unsigned int i = 0; i < assem._Tree.size(); i++)
			{
				unsigned int mateID = assem._Tree[i].first;
				unsigned int plinkID = assem._Mate[mateID].getParentLinkIdx();
				unsigned int clinkID = assem._Mate[mateID].getChildLinkIdx();
				unsigned int dofIdx;

				adJTransdqdpk[clinkID] = adJTransdqdpk[plinkID];
				adadJJdqdpdqkdpl[clinkID] = adadJJdqdpdqkdpl[plinkID];
				adJTransd2qdpkdpl[clinkID] = adJTransd2qdpkdpl[plinkID];
				for (unsigned int j = 0; j < state.getJointStateByMateIndex(mateID)._dof; j++)
				{
					dofIdx = state.getAssemIndex(mateID) + j;
					for (int k = 0; k < pN; k++)
					{
						adJTransdqdpk[clinkID][k] += SE3::adTranspose(state.getJointState(mateID)._accumulatedJ.col(j))*dqdp(dofIdx, k);
						for (int l = 0; l < pN; l++)
						{
							adadJJdqdpdqkdpl[clinkID][k][l] += SE3::adTranspose(adJTransdqdpk[clinkID][k].transpose()*state.getJointState(mateID)._accumulatedJ.col(j))*dqdp(dofIdx, l);
							if (d2qdp2.size() != 0)
								adJTransd2qdpkdpl[clinkID][k][l] += SE3::adTranspose(state.getJointState(mateID)._accumulatedJ.col(j))*d2qdp2[k](dofIdx, l);
						}
					}
				}

				Tacci = state.getJointStateByMateIndex(mateID)._accumulatedT;
				AdTransTacci = SE3::Ad(Tacci).transpose();
				for (int k = 0; k < pN; k++)
				{
					d2extForcedp2[clinkID][k] = d2extForcedp2[clinkID][k];
					for (int l = 0; l < pN; l++)
					{
						d2extForcedp2[clinkID][k].col(l) += adJTransdqdpk[clinkID][k] * dextForcedp[clinkID].col(l) +
							adJTransdqdpk[clinkID][l] * dextForcedp[clinkID].col(k) +
							adJTransdqdpk[clinkID][k] * adJTransdqdpk[clinkID][l] * extForce[clinkID] +
							adJTransd2qdpkdpl[clinkID][k][l] * extForce[clinkID] +
							adadJJdqdpdqkdpl[clinkID][k][l] * extForce[clinkID];
					}
					d2extForcedp2[clinkID][k] = AdTransTacci * d2extForcedp2[clinkID][k];
				}

				for (int k = 0; k < pN; k++)
				{
					dextForcedp[clinkID].col(k) += adJTransdqdpk[clinkID][k] * extForce[clinkID];
				}
				dextForcedp[clinkID] = AdTransTacci * dextForcedp[clinkID];
			}
		}


		dFbdp[assem._endeffectorLink] = dextForcedp[assem._endeffectorLink];
		d2Fbdp2[assem._endeffectorLink] = d2extForcedp2[assem._endeffectorLink];
		for (unsigned int i = 0; i < assem._Tree.size(); i++)
		{
			unsigned int mateID = assem._Tree[assem._Tree.size() - i - 1].first;
			unsigned int plinkID = assem._Mate[mateID].getParentLinkIdx();
			unsigned int clinkID = assem._Mate[mateID].getChildLinkIdx();
			unsigned int dofIdx;

			Tacci = state.getJointStateByMateIndex(mateID)._accumulatedT;
			AdTransTacci = SE3::Ad(Tacci).transpose();

			currFb = AdTransTacci*state.getJointStateByMateIndex(mateID)._constraintF;
			
			if (dFbdp[clinkID].rows() == 0)
				currdFbdp.setZero(6, pN);
			else
				currdFbdp = dFbdp[clinkID];
			if (d2Fbdp2[clinkID].size() == 0)
			{
				for (int k = 0; k < pN; k++)
					currd2Fbdp2[k].setZero(6, pN);
			}
			else
				currd2Fbdp2 = d2Fbdp2[clinkID];
			for (unsigned int j = 0; j < state.getJointStateByMateIndex(mateID)._dof; j++)
			{
				dofIdx = state.getAssemIndex(mateID) + state.getJointStateByMateIndex(mateID)._dof - j - 1;
				if (j == state.getJointStateByMateIndex(mateID)._dof - 1) invAdTranseSi1qi1 = SE3::InvAd(state.getJointStateByMateIndex(mateID)._T[state.getJointStateByMateIndex(mateID)._dof - j - 1]).transpose();
				else invAdTranseSi1qi1 = SE3::Ad(state.getJointStateByMateIndex(mateID)._T[state.getJointStateByMateIndex(mateID)._dof - j - 1].inverse() * state.getJointStateByMateIndex(mateID)._T[state.getJointStateByMateIndex(mateID)._dof - j - 2]).transpose();

				Si1 = assem._socMate[mateID]._axes.col(state.getJointStateByMateIndex(mateID)._dof - j - 1);
				adTransSi1 = -SE3::adTranspose(Si1);

				if (j == 0)
				{
					currdFbdp += (Matrix6&)assem._socLink[clinkID]._G*dVbdotdp[clinkID];
					for (int k = 0; k < pN; k++)
					{
						currdFbdp.col(k) -= SE3::adTranspose(dVbdp[clinkID].col(k))*((Matrix6&)assem._socLink[clinkID]._G*Vb[clinkID]);
					}
					currdFbdp += -SE3::adTranspose(Vb[clinkID])*(Matrix6&)assem._socLink[clinkID]._G*dVbdp[clinkID] +
						dextForcedp[clinkID];
				}
				if (j == 0)
				{
					for (int k = 0; k < pN; k++)
					{
						currd2Fbdp2[k] += (Matrix6&)assem._socLink[clinkID]._G*d2Vbdotdp2[clinkID][k]
							- SE3::adTranspose(Vb[clinkID])*assem._socLink[clinkID]._G*d2Vbdp2[clinkID][k]
							+ d2extForcedp2[clinkID][k];

						for (int l = 0; l <= k; l++)
						{
							currd2Fbdp2[k].col(l) -= (SE3::adTranspose(d2Vbdp2[clinkID][k].col(l))*((Matrix6&)assem._socLink[clinkID]._G*Vb[clinkID]))
								+ (SE3::adTranspose(dVbdp[clinkID].col(l))*((Matrix6&)assem._socLink[clinkID]._G*dVbdp[clinkID].col(k)))
								+ (SE3::adTranspose(dVbdp[clinkID].col(k))*((Matrix6&)assem._socLink[clinkID]._G*dVbdp[clinkID].col(l)));
							if (l != k)
							{
								currd2Fbdp2[l].col(k) = currd2Fbdp2[k].col(l);
							}
						}

					}
				}

				dtaudp.row(dofIdx) = Si1.transpose()*currdFbdp + assem.getJointPtrByMateIndex(mateID)->getConstDamper()(j)*dqdotdp.row(dofIdx);
				for (int k = 0; k < pN; k++)
				{
					d2taudp2[dofIdx].row(k) = Si1.transpose()*currd2Fbdp2[k];
				}

				pastFb = currFb;
				pastdFbdp = currdFbdp;

				currFb = invAdTranseSi1qi1*currFb;
				currdFbdp = invAdTranseSi1qi1*currdFbdp + (invAdTranseSi1qi1*(adTransSi1*pastFb))*dqdp.row(dofIdx);

				for (int k = 0; k < pN; k++)
				{
					currd2Fbdp2[k] = invAdTranseSi1qi1*(currd2Fbdp2[k] + adTransSi1*pastdFbdp*dqdp(dofIdx, k)) +
						(invAdTranseSi1qi1*(adTransSi1*pastdFbdp.col(k)))*dqdp.row(dofIdx) +
						(invAdTranseSi1qi1*(adTransSi1*(adTransSi1*pastFb)))*dqdp.row(dofIdx)*dqdp(dofIdx, k);
					if (d2qdp2.size() != 0)
					{
						currd2Fbdp2[k] += (invAdTranseSi1qi1*(adTransSi1*pastFb))*d2qdp2[k].row(dofIdx);
					}
				}
			}
			dFbdp[plinkID] = currdFbdp;
			d2Fbdp2[plinkID] = currd2Fbdp2;
		}



		return pair<MatrixX, vector<MatrixX>>(dtaudp, d2taudp2);
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
		//	J_a[i] means articulated inertia of child link of i-th mate.
		vector<Matrix6, Eigen::aligned_allocator<Matrix6>>	J_a(assem._Tree.size());
		//	Bias force applied on child link of i-th mate.
		vector<dse3, Eigen::aligned_allocator<dse3>>	bias(assem._Tree.size());
		//	Inertia of child link of i-th mate w.r.t. spatial frame.
		Inertia linkInertia = assem._socLink[cLinkIdx]._G.getTransformed(state.getJointStateByMateIndex(assem._Tree[lastMateIdx].first)._accumulatedT, true);


		J_a[lastMateIdx] = linkInertia;
		bias[lastMateIdx] = -SE3::adTranspose(state.getLinkState(cLinkIdx)._V) * linkInertia * state.getLinkState(cLinkIdx)._V;
		if (!extF[lastMateIdx + 1].isZero())
			bias[lastMateIdx] -= SE3::InvAd(state.getLinkState(cLinkIdx)._T).transpose() * extF[lastMateIdx + 1];

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
			Ja_S = (Matrix6)J_a[idx] * jStat._accumulatedJ;
			inv_ST_Ja_S = (jStat._accumulatedJ.transpose()*Ja_S).inverse();
			ad_V_S_qdot = SE3::ad(state.getLinkState(cLinkIdx)._V) * (jStat._accumulatedJ * jStat.getqdot());

			//	Articulated inertia of parent link of 'idx'-th mate.
			J_a[idx - 1] =
				static_cast<Matrix6&>(linkInertia)
				+ J_a[idx]
				- Ja_S * inv_ST_Ja_S * Ja_S.transpose();
			//	Bias force of parent link of 'idx'-th mate.
			bias[idx - 1] =
				-SE3::adTranspose(state.getLinkState(pLinkIdx)._V) * linkInertia * state.getLinkState(pLinkIdx)._V
				+ bias[idx]
				+ J_a[idx] * ad_V_S_qdot
				+ Ja_S * inv_ST_Ja_S * (jStat._tau - Ja_S.transpose() * ad_V_S_qdot - jStat._accumulatedJ.transpose()*bias[idx]);
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
			State::JointState&	jStat = state.getJointStateByMateIndex(assem._Tree[idx].first);

			//	Pre-calculate frequently used variables.
			//	TODO: Avoid duplicated calculation. (Most of these need only a single calculation among back/forward iteration)
			Ja_S = (Matrix6)J_a[idx] * jStat._accumulatedJ;
			inv_ST_Ja_S = (jStat._accumulatedJ.transpose()*Ja_S).inverse();
			ad_V_S_qdot = SE3::ad(state.getLinkState(cLinkIdx)._V) * (jStat._accumulatedJ * jStat.getqdot());
			 
			VectorX qddot =
				inv_ST_Ja_S * (jStat._tau - Ja_S.transpose()*(state.getLinkState(pLinkIdx)._VDot + ad_V_S_qdot) - jStat._accumulatedJ.transpose()*bias[idx]);
			state.setJointqddot(assem._Tree[idx].first, qddot);

			state.getLinkState(cLinkIdx)._VDot = jStat._accumulatedJ * qddot + state.getLinkState(pLinkIdx)._VDot + ad_V_S_qdot;
		}

	}

}