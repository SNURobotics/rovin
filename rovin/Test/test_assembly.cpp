#include <iostream>
#include <conio.h>
#include <ctime>

#include <rovin/Math/Inertia.h>
#include <rovin/Math/Constant.h>
#include <rovin/Math/LinearAlgebra.h>
#include <rovin/Math/LieGroup.h>
#include <rovin/Model/Assembly.h>
#include <rovin/Model/RevoluteJoint.h>
#include <rovin/Dynamics/Kinematics.h>

#include <rovin/Renderer/SimpleOSG.h>

using namespace std;
using namespace rovin::Math;
using namespace rovin::Model;

AssemblyPtr fourBar;

void Modeling();

int main()
{
	/*
	so3 w;
	w << 1, 2, 3;
	cout << SO3::Exp_new(w, 5) << endl;
	PERFORM_TEST(SO3::Exp_new(w, 5);, 1e+7);
	cout << SO3::Exp(w, 5) << endl;
	PERFORM_TEST(SO3::Exp(w, 5);, 1e+7);
	
	se3 S;
	S << 1, 2, 3, 4, 5, 6;
	cout << SE3::Exp(S, 5) << endl;
	PERFORM_TEST(SE3::Exp(S, 5);, 1e+7);
	cout << SE3::Exp_new(S, 5) << endl;
	PERFORM_TEST(SE3::Exp_new(S, 5);, 1e+7);
	*/

	Modeling();

	StatePtr state = fourBar->makeState();

	vector< string > activeJoint;
	activeJoint.push_back("J1");
	activeJoint.push_back("J2");
	activeJoint.push_back("J3");

	state->addActiveJoint(activeJoint);

	////////////////////////
	VectorX q(3);
	q.setRandom();
	state->setActiveJointq(q);
	////////////////////////

	cout << q << endl;

	//rovin::Kinematics::solveClosedLoopConstraint(*fourBar, *state);
	rovin::Kinematics::solveForwardKinematics(*fourBar, *state);

	//cout << rovin::Kinematics::computeJacobian(*fourBar, *state, "L4", "L1") << endl;

	//PERFORM_TEST(B = pinv(A);, 1e+5);
	

	cout << state->getJointState("J1").getq() << endl;
	cout << state->getJointState("J2").getq() << endl;
	cout << state->getJointState("J3").getq() << endl;
	//cout << state->getJointState("J4").getq() << endl;
	cout << state->getLinkState("L4")._T << endl;

	///
	q << PI / 3, PI / 4, -PI / 4;
	state->setActiveJointq(q);
	rovin::Kinematics::solveForwardKinematics(*fourBar, *state);
	cout << state->getLinkState("L4")._T << endl;
	SE3 goalT = state->getLinkState("L4")._T;

	q.setRandom();
	state->setActiveJointq(q);
	rovin::Kinematics::solveForwardKinematics(*fourBar, *state);
	cout << state->getJointState("J1").getq() << endl;
	cout << state->getJointState("J2").getq() << endl;
	cout << state->getJointState("J3").getq() << endl;
	cout << state->getLinkState("L4")._T << endl;

	rovin::Kinematics::solveInverseKinematics(*fourBar, *state, goalT, "L4");
	rovin::Kinematics::solveForwardKinematics(*fourBar, *state);
	cout << state->getJointState("J1").getq() << endl;
	cout << state->getJointState("J2").getq() << endl;
	cout << state->getJointState("J3").getq() << endl;
	cout << state->getLinkState("L4")._T << endl;

	rovin::Renderer::SimpleOSG renderer(*fourBar, *state, 600, 600);
	renderer._viewer.run();

	return 0;
}

void Modeling()
{
	fourBar = AssemblyPtr(new Assembly("FourbarLinkage"));
	shared_ptr< Link > L1 = shared_ptr< Link >(new Link("L1"));
	shared_ptr< Link > L2 = shared_ptr< Link >(new Link("L2"));
	shared_ptr< Link > L3 = shared_ptr< Link >(new Link("L3"));
	shared_ptr< Link > L4 = shared_ptr< Link >(new Link("L4"));
	L1->setVisualGeometry(shared_ptr< Box >(new Box(2, 10, 2)));
	L2->setVisualGeometry(shared_ptr< Box >(new Box(6, 2, 2)));
	L3->setVisualGeometry(shared_ptr< Box >(new Box(2, 10, 2)));
	L4->setVisualGeometry(shared_ptr< Box >(new Box(6, 2, 2)));
	fourBar->addLink(L1);
	fourBar->addLink(L2);
	fourBar->addLink(L3);
	fourBar->addLink(L4);
	fourBar->addMate(shared_ptr< Joint >(new RevoluteJoint("J1")), "L1", "L2", SE3(Vector3(-6, 0, 0)), SE3(Vector3(0, 4, 0)));
	fourBar->addMate(shared_ptr< Joint >(new RevoluteJoint("J2")), "L2", "L3", SE3(Vector3(0, 4, 0)), SE3(Vector3(6, 0, 0)));
	fourBar->addMate(shared_ptr< Joint >(new RevoluteJoint("J3")), "L3", "L4", SE3(Vector3(6, 0, 0)), SE3(Vector3(0, -4, 0)));
	//fourBar->addMate(shared_ptr< Joint >(new RevoluteJoint("J4")), "L1", "L4", SE3(Vector3(6, 0, 0)), SE3(Vector3(0, 4, 0)));
	fourBar->completeAssembling("L1");
}