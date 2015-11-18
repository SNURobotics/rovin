#include <iostream>
#include <conio.h>
#include <ctime>

#include <rovin/Math/Inertia.h>
#include <rovin/Math/Constant.h>
#include <rovin/Math/LinearAlgebra.h>
#include <rovin/Math/LieGroup.h>
#include <rovin/Model/Assembly.h>
#include <rovin/Model/RevoluteJoint.h>

using namespace std;
using namespace rovin::Math;
using namespace rovin::Model;
using namespace rovin::Dynamics;

shared_ptr< System > robot;
unsigned int _DOF = 3;

void Modeling(unsigned int );

int main()
{
	srand(time(NULL));

	Modeling(_DOF);

	list< string > activeJoint;
	for (unsigned int i = 1; i <= _DOF; i++)
	{
		activeJoint.push_back("J" + to_string(i));
	}

	State state(*robot, activeJoint);

	////////////////////////
	VectorX q(_DOF);
	q.setRandom();
	state.setActiveJoint_q(q);
	////////////////////////

	//fourBar->Solve_Closedloop_Constraint(state);

	PERFORM_TEST(robot->Forward_Kinematics(state), 1e+6);

	return 0;
}

void Modeling(unsigned int DOF)
{
	shared_ptr< Assembly > asem = shared_ptr< Assembly >(new Assembly());

	shared_ptr< Link > base = shared_ptr< Link >(new Link("L0"));
	asem->addLink(base);
	for (unsigned int i = 1; i <= DOF; i++)
	{
		shared_ptr< Link > L1 = shared_ptr< Link >(new Link("L"+to_string(i)));
		asem->addLink(L1);
		asem->addJoint(shared_ptr< Joint >(new RevoluteJoint("J" + to_string(i))), "L" + to_string(i - 1), "L" + to_string(i), SE3(), SE3());
	}

	robot = shared_ptr< System >(new System(asem, "L0"));
}