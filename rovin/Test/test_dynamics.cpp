#include <iostream>
#include <conio.h>
#include <ctime>

#include <rovin/Math/Inertia.h>
#include <rovin/Math/Common.h>
#include <rovin/Math/Constant.h>
#include <rovin/Math/LieGroup.h>
#include <rovin/Model/Assembly.h>
#include <rovin/Model/RevoluteJoint.h>
#include <rovin/Dynamics/Kinematics.h>
#include <rovin/Dynamics/Dynamics.h>

#include <rovin/Renderer/SimpleOSG.h>
#include <rovin/utils/Diagnostic.h>

#include <rovin/Test/test_OpenChainAssem.h>

using namespace std;
using namespace rovin::Math;
using namespace rovin::Model;
using namespace rovin::Renderer;

socAssemblyPtr openchain;

void Modeling();


class IDFunction : public Function
{
public:
	IDFunction() {};
	VectorX func(const VectorX& x) const
	{
		//state->setActiveJointq(x.head(dof));
		//state->setActiveJointqdot(x.segment(dof, dof));
		//state->setActiveJointqddot(x.tail(dof));

		VectorX q(dof);
		VectorX dq(dof);
		VectorX ddq(dof);
		q = x + 0.5*x.cwiseAbs2() + q0;
		dq = dp + x.cwiseProduct(dp) + dq0;
		ddq = x.cwiseProduct(ddp) + dp.cwiseAbs2() + ddp + ddq0;
		state->setJointq(State::TARGET_JOINT::ACTIVEJOINT, q);
		state->setJointqdot(State::TARGET_JOINT::ACTIVEJOINT, dq);
		state->setJointqddot(State::TARGET_JOINT::ACTIVEJOINT, ddq);
		//	solve inv. dyn.
		//rovin::Kinematics::solveForwardKinematics(*openchain, *state, rovin::Kinematics::VELOCITY | rovin::Kinematics::ACCELERATION);
		rovin::Dynamics::solveInverseDynamics(*openchain, *state);
		return state->getJointTorque(State::TARGET_JOINT::STATEJOINT);
		//return SE3::InvAd(state->getJointState("J2")._accumulatedT)*state->getLinkState("L3")._V;
		//state->getLinkState(1)._V;
		//return SE3::InvAd(state->getJointState("J5")._accumulatedT)*state->getLinkState("L6")._VDot;
	}
	unsigned int dof;
	VectorX q0;
	VectorX dq0;
	VectorX ddq0;
	VectorX dp;
	VectorX ddp;
	socAssemblyPtr socAssem;
	StatePtr state;
};

int main()
{
	Modeling();

	StatePtr state = openchain->makeState();
	StatePtr stateNumeric = openchain->makeState();
	FunctionPtr obj_f = FunctionPtr(new IDFunction());


	//	initialization
	unsigned int dof = state->getDOF(State::TARGET_JOINT::ACTIVEJOINT);
	VectorX p(dof),dp(dof),ddp(dof);
	p.setRandom();
	dp.setRandom();
	ddp.setRandom();

	VectorX  q0, dq0, ddq0, tau;
	//	set q, qdot, qddot as uniform random number between -1 and 1
	//q = VectorX::Random(dof, 1);
	//dq = VectorX::Random(dof, 1);
	//ddq = VectorX::Random(dof, 1);
	q0 = VectorX::Random(dof, 1);
	dq0 = VectorX::Random(dof, 1);
	ddq0 = VectorX::Random(dof, 1);


	state->setJointq(State::TARGET_JOINT::ACTIVEJOINT, q0+p+0.5*p.cwiseProduct(p));
	state->setJointqdot(State::TARGET_JOINT::ACTIVEJOINT, dp+dp.cwiseProduct(p)+dq0);
	state->setJointqddot(State::TARGET_JOINT::ACTIVEJOINT, ddp+dp.cwiseAbs2()+ddp.cwiseProduct(p)+ddq0);
	//	solve inv. dyn.
	rovin::Dynamics::solveInverseDynamics(*openchain, *state);

	//	print out result
	cout << "=== Inverse Dynamics ===" << endl;
	cout << "q	: " << (q0 + p + 0.5*p.cwiseProduct(p)).transpose() << endl;
	cout << "qdot	: " << (dp + dp.cwiseProduct(p) + dq0).transpose() << endl;
	cout << "qddot	: " << (ddp + dp.cwiseAbs2() + ddp.cwiseProduct(p) + ddq0).transpose() << endl;
	cout << endl << "tau	: " << state->getJointTorque(State::TARGET_JOINT::STATEJOINT).transpose() << endl;

	rovin::Dynamics::solveForwardDynamics(*openchain, *state);
	cout << endl << "tau	: " << state->getJointqddot(State::TARGET_JOINT::STATEJOINT).transpose() << endl;
	

	//cout << "=== differentiate inverse dynamics ===" << endl;
	//pair<MatrixX, vector<MatrixX>> tauDeriv;
	//
	//// set dqdp, dqdotdp, dqddotdp
	//MatrixX zeroMatrix(dof, dof);
	//zeroMatrix.setZero();
	//MatrixX identity(dof, dof);
	//identity.setIdentity();

	//int pN = dof;
	//MatrixX dqdp(dof, dof);
	//dqdp = (VectorX::Ones(dof) + p).asDiagonal();
	//MatrixX dqdotdp(dof, dof);
	//dqdotdp = (dp).asDiagonal();
	//MatrixX dqddotdp(dof, dof);
	//dqddotdp = (ddp).asDiagonal();
	//vector<MatrixX> d2qdp2(pN, MatrixX::Zero(dof, pN));
	//for (int i = 0; i < pN; i++)
	//{
	//	d2qdp2[i](i, i) = 1.0;
	//}
	//PERFORM_TEST(
	//	state->addJointq(State::TARGET_JOINT::ACTIVEJOINT, VectorX::Zero(dof));
	//	tauDeriv = rovin::Dynamics::differentiateInverseDynamics(*openchain, *state, dqdp, dqdotdp, dqddotdp, false, d2qdp2);
	//	, 1);
	//
	//cout << "analytic jacobian" << endl;
	//cout << tauDeriv.first << endl;
	//cout << "analytic hessian" << endl;
	//cout << tauDeriv.second[4] << endl;

	//
	//// Numerical derivative
	//std::static_pointer_cast<IDFunction>(obj_f)->dof = dof;
	//std::static_pointer_cast<IDFunction>(obj_f)->socAssem = openchain;
	//std::static_pointer_cast<IDFunction>(obj_f)->state = stateNumeric;
	//std::static_pointer_cast<IDFunction>(obj_f)->q0 = q0;
	//std::static_pointer_cast<IDFunction>(obj_f)->dq0 = dq0;
	//std::static_pointer_cast<IDFunction>(obj_f)->ddq0 = ddq0;
	//std::static_pointer_cast<IDFunction>(obj_f)->dp = dp;
	//std::static_pointer_cast<IDFunction>(obj_f)->ddp = ddp;
	//MatrixX NumericJacobian;
	//vector< MatrixX >NumericJHessian;
	//PERFORM_TEST(
	//	NumericJacobian = obj_f->Jacobian(p);
	//	NumericJHessian = obj_f->Hessian(p);
	//	, 1);

	//cout << "=== numerical jacobian ===" << endl;
	//cout << NumericJacobian << endl;

	//cout << "=== numerical hessian ===" << endl;
	//cout << NumericJHessian[4] << endl;

	//for (unsigned int i = 0; i < dof; i++)
	//{
	//	cout << "dtaudp_" << i+1 << " = " << tauDeriv.first[i].transpose() << endl;
	//}
	//Real epsilon = 1.0e-6;
	//VectorX q1, q2, dq1, dq2, ddq1, ddq2, tau1, tau2;
	//MatrixX I(dof, dof);
	//I.setIdentity();
	//StatePtr state1 = openchain->makeState();
	//StatePtr state2 = openchain->makeState();
	//for (unsigned int i = 0; i < dof; i++)
	//{
	//	q1 = q + I.col(i);
	//	q2 = q - I.col(i);
	//	dq1 = dq + I.col(i);
	//	dq2 = dq - I.col(i);
	//	ddq1 = ddq + I.col(i);
	//	ddq2 = ddq - I.col(i);
	//	state1->setActiveJointq(q1);
	//	state1->setActiveJointqdot(dq1);
	//	state1->setActiveJointqddot(ddq1);
	//	state1->setActiveJointq(q1);
	//	state1->setActiveJointqdot(dq1);
	//	state1->setActiveJointqddot(ddq1);
	//	rovin::Dynamics::solveInverseDynamics(*openchain, *state1);
	//	rovin::Dynamics::solveInverseDynamics(*openchain, *state2);
	//}

	//	solve forw. dyn. with result of inv. dyn.
	//rovin::Dynamics::solveForwardDynamics(*openchain, *state);

	//cout << endl << "=== Forward Dynamics ===" << endl;
	//cout << "tau	: " << state->getJointTorque(State::TARGET_JOINT::STATEJOINT).transpose() << endl;
	//cout << "q	: " << state->getJointq(State::TARGET_JOINT::ACTIVEJOINT).transpose() << endl;
	//cout << "qdot	: " << state->getJointqdot(State::TARGET_JOINT::ACTIVEJOINT).transpose() << endl;
	//cout << endl << "qddot	: " << state->getJointqddot(State::TARGET_JOINT::ACTIVEJOINT).transpose() << endl;

	//rovin::Kinematics::solveForwardKinematics(*openchain, *state);
	//SimpleOSG renderer(*openchain, *state, 600, 600);
	//renderer._viewer.run();

	return 0;
}

void Modeling()
{
	openchain = socAssemblyPtr(new socAssembly("FourbarLinkage"));

	shared_ptr< Link > L1 = shared_ptr< Link >(new Link("L1"));
	shared_ptr< Link > L2 = shared_ptr< Link >(new Link("L2"));
	shared_ptr< Link > L3 = shared_ptr< Link >(new Link("L3"));
	shared_ptr< Link > L4 = shared_ptr< Link >(new Link("L4"));
	shared_ptr< Link > L5 = shared_ptr< Link >(new Link("L5"));
	shared_ptr< Link > L6 = shared_ptr< Link >(new Link("L6"));
	L1->setVisualGeometry(shared_ptr< Box >(new Box(0.7, 1.5, 0.7)));
	L2->setVisualGeometry(shared_ptr< Box >(new Box(0.7, 1.5, 0.7)));
	L3->setVisualGeometry(shared_ptr< Box >(new Box(0.7, 1.5, 0.7)));
	L4->setVisualGeometry(shared_ptr< Box >(new Box(0.7, 1.5, 0.7)));
	L5->setVisualGeometry(shared_ptr< Box >(new Box(0.7, 1.5, 0.7)));
	L6->setVisualGeometry(shared_ptr< Box >(new Box(0.7, 1.5, 0.7)));
	openchain->addLink(L1);
	openchain->addLink(L2);
	openchain->addLink(L3);
	openchain->addLink(L4);
	openchain->addLink(L5);
	openchain->addLink(L6);
	openchain->addMate(shared_ptr< Joint >(new RevoluteJoint("J1")), "L1", "L2", SE3(Vector3(1, 0, 0)), SE3(Vector3(1, 0, 0)));
	openchain->addMate(shared_ptr< Joint >(new RevoluteJoint("J2")), "L2", "L3", SE3(Vector3(1, 0, 0)), SE3(Vector3(1, 0, 0)));
	openchain->addMate(shared_ptr< Joint >(new RevoluteJoint("J3")), "L3", "L4", SE3(Vector3(1, 0, 0)), SE3(Vector3(1, 0, 0)));
	openchain->addMate(shared_ptr< Joint >(new RevoluteJoint("J4")), "L4", "L5", SE3(Vector3(1, 0, 0)), SE3(Vector3(1, 0, 0)));
	openchain->addMate(shared_ptr< Joint >(new RevoluteJoint("J5")), "L5", "L6", SE3(Vector3(1, 0, 0)), SE3(Vector3(1, 0, 0)));

	//openchain->getJointPtr("J1")->setConstDamper(VectorX::Ones(1));
	//openchain->getJointPtr("J2")->setConstDamper(VectorX::Ones(1));
	//openchain->getJointPtr("J3")->setConstDamper(VectorX::Ones(1));
	//openchain->getJointPtr("J4")->setConstDamper(VectorX::Ones(1));
	//openchain->getJointPtr("J5")->setConstDamper(VectorX::Ones(1));

	//openchain->getJointPtr("J1")->setConstFriction(VectorX::Ones(1));
	//openchain->getJointPtr("J2")->setConstFriction(VectorX::Ones(1));
	//openchain->getJointPtr("J3")->setConstFriction(VectorX::Ones(1));
	//openchain->getJointPtr("J4")->setConstFriction(VectorX::Ones(1));
	//openchain->getJointPtr("J5")->setConstFriction(VectorX::Ones(1));

	openchain->completeAssembling("L1");
}
