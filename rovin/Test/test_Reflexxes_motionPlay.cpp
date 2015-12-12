#include <rovin/Renderer/OSG_simpleRender.h>
#include <rovin/Math/GaussianQuadrature.h>
#include <rovin/utils/utils.h>
#include <rovin/Reflexxes/Core.h>
#include <omp.h>
#include <chrono>

#include "efortRobot.h"
#include "test_functions.h"

using namespace std;
using namespace rovin::Math;
using namespace rovin::Model;
using namespace rovin::Renderer;
using namespace rovin::utils;

using namespace std::chrono;

efortRobot gAssem;
const double timeStep = 0.02;

Reflexxes::ReflexxesWrapper setupReflexxesProblem(const socAssembly&	assem);
int main()
{
	auto ReflexxSolver = setupReflexxesProblem(gAssem);
	auto traj = ReflexxSolver.solve();
	auto torque = calcTorque(gAssem, ReflexxSolver.getResultPos(), ReflexxSolver.getResultVel(), ReflexxSolver.getResultAcc());

	auto effort = calcEffort(torque);
	auto energy = calcEnergy(gAssem, ReflexxSolver.getResultVel(), ReflexxSolver.getResultAcc(), torque);

	cout << "Effort :	" << effort.sum()*timeStep << endl;
	cout << "Energy :	" << energy.sum()*timeStep << endl;
	//writeText(traj.transpose(), "reflexxTraj.txt");


	//	Open renderer window
	StatePtr state = gAssem.makeState();
	OSG_simpleRender renderer(gAssem, *state, 600, 600);
	renderer.getViewer().realize();

	double t = omp_get_wtime();
	while (1)
	{
		Points	trajPoints;
		renderer.addGeometry(trajPoints);
		for (int i = 0; i < traj.cols(); i++)
			while (true)
			{
				if (omp_get_wtime() - t >= timeStep)
				{
					state->setJointq(State::ACTIVEJOINT, traj.col(i));
					rovin::Kinematics::solveForwardKinematics(gAssem, *state, State::LINKS_POS);
					trajPoints.push_back(eigen2osgVec<osg::Vec3>(state->getLinkState(6)._T.getPosition()));
					renderer.updateFrame();
					t = omp_get_wtime();
					break;
				}
			}
		renderer.removeGeometry(trajPoints);
	}

	renderer.getViewer().run();
	return 0;
}

Reflexxes::ReflexxesWrapper setupReflexxesProblem(const socAssembly & assem)
{
	//	Construct Reflexx class
	StatePtr state = assem.makeState();
	Reflexxes::ReflexxesWrapper		ReflexxSolver(state->getDOF(State::STATEJOINT), timeStep);
	//	Set initial and fial state (otherwise zero).
	ReflexxSolver._q0.setZero();
	ReflexxSolver._qf.setOnes();
	//	Set kinematic limit.
	for (unsigned int i = 0; i < state->getDOF(State::STATEJOINT); i++)
	{
		ReflexxSolver._dqLim[i] = assem.getJointPtrByMateIndex(i)->getLimitVelUpper().operator[](0);
		ReflexxSolver._ddqLim[i] = assem.getJointPtrByMateIndex(i)->getLimitAccUpper().operator[](0);
	}
	return ReflexxSolver;
}
