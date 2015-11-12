#include <iostream>
#include <conio.h>

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

shared_ptr< System > sys;

void Modeling();

int main()
{
	Modeling();

	list< string > activeJoint;
	activeJoint.push_back("J1");

	State state(*sys, activeJoint);
	VectorX q(1);
	q << 1;
	state.setActiveJoint_q(q);
	cout << sys->Closedloop_Constraint_Function(state) << endl;
		
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
	L1->setVisualGeometry(shared_ptr< Box >(new Box(10, 2, 2)));
	L2->setVisualGeometry(shared_ptr< Box >(new Box(2, 2, 6)));
	L3->setVisualGeometry(shared_ptr< Box >(new Box(10, 2, 2)));
	L4->setVisualGeometry(shared_ptr< Box >(new Box(2, 2, 6)));
	asem->addLink(L1);
	asem->addLink(L2);
	asem->addLink(L3);
	asem->addLink(L4);
	asem->addJoint(shared_ptr< Joint >(new RevoluteJoint("J1")), "L1", "L2", SE3(Vector3(0, -6, 0)), SE3(Vector3(0, 0, 4)));
	asem->addJoint(shared_ptr< Joint >(new RevoluteJoint("J2")), "L2", "L3", SE3(Vector3(0, 0, 4)), SE3(Vector3(0, 6, 0)));
	asem->addJoint(shared_ptr< Joint >(new RevoluteJoint("J3")), "L3", "L4", SE3(Vector3(0, 6, 0)), SE3(Vector3(0, 0, -4)));
	asem->addJoint(shared_ptr< Joint >(new RevoluteJoint("J4")), "L1", "L4", SE3(Vector3(0, 6, 0)), SE3(Vector3(0, 0, 4)));

	sys = shared_ptr< System >(new System(asem, "L1"));
}