#include <rovin/Renderer/OSG_simpleRender.h>
#include <rovin/Math/GaussianQuadrature.h>
#include <rovin/utils/utils.h>
#include <rovin/Reflexxes/Core.h>

#include "efortRobot.h"


using namespace std;
using namespace rovin::Math;
using namespace rovin::Model;
using namespace rovin::Renderer;
using namespace rovin::utils;

efortRobot gAssem;
const double timeStep = 0.01;
int main()
{
	StatePtr state = gAssem.makeState();

	Reflexxes::ReflexxesWrapper		ReflexxSolver(state->getDOF(State::STATEJOINT), timeStep);
	ReflexxSolver._q0.setZero();
	ReflexxSolver._qf.setOnes();
	for (unsigned int i = 0; i < state->getDOF(State::STATEJOINT); i++)
	{
		ReflexxSolver._dqLim[i] = gAssem.getJointPtrByMateIndex(i)->getLimitVelUpper().operator[](0);
		ReflexxSolver._ddqLim[i] = gAssem.getJointPtrByMateIndex(i)->getLimitAccUpper().operator[](0);
	}
	auto traj = ReflexxSolver.solve();

	rovin::Kinematics::solveForwardKinematics(gAssem, *state, State::LINKS_POS);
	OSG_simpleRender renderer(gAssem, *state, 600, 600);
	renderer.getViewer().realize();

	//writeText(traj.transpose(), "reflexxTraj.txt");

	double c = clock();

	while (1)
	{
		Points	trajPoints;
		renderer.addGeometry(trajPoints);
		for (int i = 0; i < traj.cols(); i++)
			while (true)
			{
				if (clock() - c >= timeStep * 1000)
				{
					state->setJointq(State::ACTIVEJOINT, traj.col(i));
					rovin::Kinematics::solveForwardKinematics(gAssem, *state, State::LINKS_POS);
					trajPoints.push_back(eigen2osgVec<osg::Vec3>(state->getLinkState(6)._T.getPosition()));
					renderer.updateFrame();
					c = clock();
					break;
				}
			}
		renderer.removeGeometry(trajPoints);
	}

	renderer.getViewer().run();
	return 0;
}