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
using namespace rovin::Renderer;

socAssemblyPtr openchain;

void Modeling();

int main()
{
	Modeling();

	StatePtr state = openchain->makeState();

	VectorX q(3);
	q << PI / 2, PI / 2, PI / 2;
	state->setActiveJointq(q);

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
	L1->setVisualGeometry(shared_ptr< Box >(new Box(2, 10, 2)));
	L2->setVisualGeometry(shared_ptr< Box >(new Box(6, 2, 2)));
	L3->setVisualGeometry(shared_ptr< Box >(new Box(2, 10, 2)));
	L4->setVisualGeometry(shared_ptr< Box >(new Box(6, 2, 2)));
	openchain->addLink(L1);
	openchain->addLink(L2);
	openchain->addLink(L3);
	openchain->addLink(L4);
	openchain->addMate(shared_ptr< Joint >(new RevoluteJoint("J1")), "L1", "L2", SE3(Vector3(-6, 0, 0)), SE3(Vector3(0, 4, 0)));
	openchain->addMate(shared_ptr< Joint >(new RevoluteJoint("J2")), "L2", "L3", SE3(Vector3(0, 4, 0)), SE3(Vector3(6, 0, 0)));
	openchain->addMate(shared_ptr< Joint >(new RevoluteJoint("J3")), "L3", "L4", SE3(Vector3(6, 0, 0)), SE3(Vector3(0, -4, 0)));

	openchain->completeAssembling("L1");
}