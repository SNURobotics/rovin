#include <iostream>
#include <conio.h>
#include <ctime>

#include <rovin/Math/Inertia.h>
#include <rovin/Math/Constant.h>
#include <rovin/Math/LieGroup.h>
#include <rovin/Model/Assembly.h>
#include <rovin/Model/RevoluteJoint.h>
#include <rovin/Dynamics/Kinematics.h>
#include <rovin/Dynamics/Dynamics.h>

#include <rovin/Renderer/SimpleOSG.h>

using namespace std;
using namespace rovin::Math;
using namespace rovin::Model;
using namespace rovin::Renderer;

socAssemblyPtr openchain;

void Modeling();

int main()
{
	SE3 goalT;

	Modeling();

	StatePtr state = openchain->makeState();

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
	rovin::Dynamics::solveInverseDynamics(*openchain, *state);

	//	print out result
	cout << "=== Inverse Dynamics ===" << endl;
	cout << "q	: " << q.transpose() << endl;
	cout << "qdot	: " << dq.transpose() << endl;
	cout << "qddot	: " << ddq.transpose() << endl;
	cout << endl << "tau	: " << state->getJointTorque(State::TARGET_JOINT::STATEJOINT).transpose() << endl;

	//	solve forw. dyn. with result of inv. dyn.
	rovin::Dynamics::solveForwardDynamics(*openchain, *state);

	cout << endl << "=== Forward Dynamics ===" << endl;
	cout << "tau	: " << state->getJointTorque(State::TARGET_JOINT::STATEJOINT).transpose() << endl;
	cout << "q	: " << state->getJointq(State::TARGET_JOINT::ACTIVEJOINT).transpose() << endl;
	cout << "qdot	: " << state->getJointqdot(State::TARGET_JOINT::ACTIVEJOINT).transpose() << endl;
	cout << endl << "qddot	: " << state->getJointqddot(State::TARGET_JOINT::ACTIVEJOINT).transpose() << endl;

	rovin::Kinematics::solveForwardKinematics(*openchain, *state);
	SimpleOSG renderer(*openchain, *state, 600, 600);
	renderer._viewer.run();

	return 0;
}

void Modeling()
{
	openchain = socAssemblyPtr(new socAssembly("FourbarLinkage"));

	shared_ptr< Link > L1 = shared_ptr< Link >(new Link("L1"));
	shared_ptr< Link > L2 = shared_ptr< Link >(new Link("L2"));
	shared_ptr< Link > L3 = shared_ptr< Link >(new Link("L3"));
	shared_ptr< Link > L4 = shared_ptr< Link >(new Link("L4"));
	L1->setVisualGeometry(shared_ptr< Box >(new Box(0.7, 1.5, 0.7)));
	L2->setVisualGeometry(shared_ptr< Box >(new Box(0.7, 1.5, 0.7)));
	L3->setVisualGeometry(shared_ptr< Box >(new Box(0.7, 1.5, 0.7)));
	L4->setVisualGeometry(shared_ptr< Box >(new Box(0.7, 1.5, 0.7)));
	openchain->addLink(L1);
	openchain->addLink(L2);
	openchain->addLink(L3);
	openchain->addLink(L4);
	openchain->addMate(shared_ptr< Joint >(new RevoluteJoint("J1")), "L1", "L2", SE3(Vector3(1, 0, 0)), SE3(Vector3(1, 0, 0)));
	openchain->addMate(shared_ptr< Joint >(new RevoluteJoint("J2")), "L2", "L3", SE3(Vector3(1, 0, 0)), SE3(Vector3(1, 0, 0)));
	openchain->addMate(shared_ptr< Joint >(new RevoluteJoint("J3")), "L3", "L4", SE3(Vector3(1, 0, 0)), SE3(Vector3(1, 0, 0)));

	openchain->completeAssembling("L1");
}