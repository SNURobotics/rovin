#include <iostream>

#include <rovin/Dynamics/Kinematics.h>
#include <rovin/Dynamics/Dynamics.h>
#include <rovin/utils/Diagnostic.h>
#include <rovin/Test/test_OpenChainAssem.h>

using namespace std;
using namespace rovin::Math;
using namespace rovin::Model;

socAssemblyPtr openchain;

void Modeling();

int main()
{
	const unsigned int DOF = 3;
	serialChain<3> myModel;
	StatePtr state = myModel.makeState();

	//	initialization
	unsigned int dof = state->getTotalJointDof();
	VectorX  q, dq, ddq, tau;
	//	set q, qdot, qddot as uniform random number between -1 and 1
	q = VectorX::Random(dof, 1);
	dq = VectorX::Random(dof, 1);
	ddq = VectorX::Random(dof, 1);
	state->setActiveJointq(q);
	state->setActiveJointqdot(dq);
	state->setActiveJointqddot(ddq);
	//	solve inv. dyn.
	rovin::Dynamics::solveInverseDynamics(myModel, *state);

	//	print out result
	cout << "=== Inverse Dynamics ===" << endl;
	cout << "q	: " << q.transpose() << endl;
	cout << "qdot	: " << dq.transpose() << endl;
	cout << "qddot	: " << ddq.transpose() << endl;
	cout << endl << "tau	: " << state->getJointTorque(State::TARGET_JOINT::STATEJOINT).transpose() << endl;

	//	solve forw. dyn. with result of inv. dyn.
	rovin::Dynamics::solveForwardDynamics(myModel, *state);

	cout << endl << "=== Forward Dynamics ===" << endl;
	cout << "tau	: " << state->getJointTorque(State::TARGET_JOINT::STATEJOINT).transpose() << endl;
	cout << "q	: " << state->getJointq(State::TARGET_JOINT::ACTIVEJOINT).transpose() << endl;
	cout << "qdot	: " << state->getJointqdot(State::TARGET_JOINT::ACTIVEJOINT).transpose() << endl;
	cout << endl << "qddot	: " << state->getJointqddot(State::TARGET_JOINT::ACTIVEJOINT).transpose() << endl;

	serialChain<3> A;
	
	VectorX temp_q(dof);
	temp_q.setRandom();
	PERFORM_TEST(
		state->addActiveJointq(temp_q);
	rovin::Dynamics::solveForwardDynamics(myModel, *state);
	, 1e+5);

	return 0;
}
