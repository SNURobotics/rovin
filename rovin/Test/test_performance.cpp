#include <iostream>
#include <conio.h>
#include <ctime>

#include <rovin/Math/Inertia.h>
#include <rovin/Math/Constant.h>
#include <rovin/Math/LinearAlgebra.h>
#include <rovin/Math/LieGroup.h>
#include <rovin/Model/Assembly.h>
#include <rovin/Model/SerialOpenChainAssembly.h>
#include <rovin/Model/RevoluteJoint.h>
#include <rovin/Dynamics/Kinematics.h>

#include <rovin/utils/Diagnostic.h>

using namespace std;
using namespace rovin::Math;
using namespace rovin::Model;

socAssemblyPtr robot;
AssemblyPtr robot_gen;

unsigned int _DOF = 60;

void Modeling(unsigned int);

int main()
{
	srand(time(NULL));

	Modeling(_DOF);
	robot_gen = static_pointer_cast<Assembly>(robot);

	list< string > activeJoint;
	for (unsigned int i = 1; i <= _DOF; i++)
	{
		activeJoint.push_back("J" + to_string(i));
	}

	StatePtr state = robot->makeState();

	////////////////////////
	VectorX q(_DOF);
	q.setRandom();
	state->setJointq(State::ACTIVEJOINT, q);
	////////////////////////

	//fourBar->Solve_Closedloop_Constraint(state);

	PERFORM_TEST(rovin::Kinematics::solveForwardKinematics(*robot, *state); , 2e+5);
	PERFORM_TEST(rovin::Kinematics::solveForwardKinematics(*robot, *state); , 2e+5);
	PERFORM_TEST(rovin::Kinematics::solveForwardKinematics(*robot, *state); , 2e+5);
	PERFORM_TEST(rovin::Kinematics::solveForwardKinematics(*robot_gen, *state); , 2e+5);
	PERFORM_TEST(rovin::Kinematics::solveForwardKinematics(*robot_gen, *state); , 2e+5);
	PERFORM_TEST(rovin::Kinematics::solveForwardKinematics(*robot_gen, *state); , 2e+5);

	return 0;
}

void Modeling(unsigned int DOF)
{
	robot = socAssemblyPtr(new socAssembly("PERFORMANCE"));

	shared_ptr< Link > base = shared_ptr< Link >(new Link("L0"));
	robot->addLink(base);
	for (unsigned int i = 1; i <= DOF; i++)
	{
		shared_ptr< Link > L1 = shared_ptr< Link >(new Link("L" + to_string(i)));
		robot->addLink(L1);
		robot->addMate(shared_ptr< Joint >(new RevoluteJoint("J" + to_string(i))), "L" + to_string(i - 1), "L" + to_string(i), SE3(), SE3());
	}

	robot->completeAssembling("L0");
}