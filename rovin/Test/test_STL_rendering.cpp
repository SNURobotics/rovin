#include <rovin/Renderer/OSG_simpleRender.h>
#include <rovin/Math/GaussianQuadrature.h>
#include <rovin/utils/utils.h>

#include "efortRobot.h"


using namespace std;
using namespace rovin::Math;
using namespace rovin::Model;
using namespace rovin::Renderer;
using namespace rovin::utils;

efortRobot gAssem;

int main()
{
	StatePtr state = gAssem.makeState();
	rovin::Kinematics::solveForwardKinematics(gAssem, *state, State::LINKS_POS);
	OSG_simpleRender renderer(gAssem, *state, 600, 600);

	renderer.getViewer().realize();
	double c = clock();
	auto q = state->getJointq(State::STATEJOINT);
	double frameRate = 120;

	Line	trajLine;
	Points	trajPoints;
	renderer.addGeometry(trajLine);
	renderer.addGeometry(trajPoints);

	while (1)
	{
		if (clock() - c >= 1000/frameRate)
		{
			q.setOnes();
			
			q /= frameRate;
			state->addJointq(State::ACTIVEJOINT, q);
			rovin::Kinematics::solveForwardKinematics(gAssem, *state, State::LINKS_POS);
			trajLine.push_back(eigen2osgVec<osg::Vec3>(state->getLinkState(6)._T.getPosition()));
			//trajPoints.push_back(eigen2osgVec<osg::Vec3>(state->getLinkState(6)._T.getPosition()));
			
			c = clock();
		}
		renderer.updateFrame();
	}
	
	renderer.getViewer().run();
	return 0;
}