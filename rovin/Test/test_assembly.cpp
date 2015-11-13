#include <iostream>
#include <conio.h>
#include <ctime>

#include <rovin/Math/Inertia.h>
#include <rovin/Math/Constant.h>
#include <rovin/Math/LieGroup.h>
#include <rovin/Model/Assembly.h>
#include <rovin/Model/RevoluteJoint.h>

#include <rovin/Dynamics/System.h>
#include <rovin/Dynamics/State.h>

using namespace std;
using namespace rovin::Math;
using namespace rovin::Model;
using namespace rovin::Dynamics;

shared_ptr< System > fourBar;

void Modeling();

int main()
{
	Modeling();

	list< string > activeJoint;
	activeJoint.push_back("J1");

	State state(*fourBar, activeJoint);

	////////////////////////
	state.getJointState("J1").q << 0.5;
	////////////////////////

	////////////////////////
	VectorX q(1);
	q << 0.5;
	state.setActiveJoint_q(q);
	////////////////////////

	fourBar->Solve_Closedloop_Constraint(state);

	cout << state.getJointState("J1").q << endl;
	cout << state.getJointState("J2").q << endl;
	cout << state.getJointState("J3").q << endl;
	cout << state.getJointState("J4").q << endl;
		
	_getch();
	return 0;
}

void Modeling()
{
	shared_ptr< Assembly > asem = shared_ptr< Assembly >(new Assembly());

	shared_ptr< Link > L1 = shared_ptr< Link >(new Link("L1"));
	shared_ptr< Link > L2 = shared_ptr< Link >(new Link("L2"));
	shared_ptr< Link > L3 = shared_ptr< Link >(new Link("L3"));
	shared_ptr< Link > L4 = shared_ptr< Link >(new Link("L4"));
	L1->setVisualGeometry(shared_ptr< Box >(new Box(2, 10, 2)));
	L2->setVisualGeometry(shared_ptr< Box >(new Box(6, 2, 2)));
	L3->setVisualGeometry(shared_ptr< Box >(new Box(2, 10, 2)));
	L4->setVisualGeometry(shared_ptr< Box >(new Box(6, 2, 2)));
	asem->addLink(L1);
	asem->addLink(L2);
	asem->addLink(L3);
	asem->addLink(L4);
	asem->addJoint(shared_ptr< Joint >(new RevoluteJoint("J1")), "L1", "L2", SE3(Vector3(-6, 0, 0)), SE3(Vector3(0, 4, 0)));
	asem->addJoint(shared_ptr< Joint >(new RevoluteJoint("J2")), "L2", "L3", SE3(Vector3(0, 4, 0)), SE3(Vector3(6, 0, 0)));
	asem->addJoint(shared_ptr< Joint >(new RevoluteJoint("J3")), "L3", "L4", SE3(Vector3(6, 0, 0)), SE3(Vector3(0, -4, 0)));
	asem->addJoint(shared_ptr< Joint >(new RevoluteJoint("J4")), "L1", "L4", SE3(Vector3(6, 0, 0)), SE3(Vector3(0, 4, 0)));

	fourBar = shared_ptr< System >(new System(asem, "L1"));
}