#include <rovin/Renderer/OSG_simpleRender.h>
#include <rovin/Math/GaussianQuadrature.h>
#include <rovin/utils/fileIO.h>

#include "efortRobot.h"


using namespace std;
using namespace rovin::Math;
using namespace rovin::Model;
using namespace rovin::Renderer;

efortRobot gAssem;

int main()
{
	StatePtr state = gAssem.makeState();
	rovin::Kinematics::solveForwardKinematics(gAssem, *state, State::LINKS_POS);
	OSG_simpleRender renderer(gAssem, *state, 600, 600);
	renderer._viewer.realize();

	

	double c = clock();
	auto q = state->getJointq(State::STATEJOINT);

	double frameRate = 120;
	while (1)
	{
		if (clock() - c >= 1000/frameRate)
		{
			q.setOnes();
			
			q /= frameRate;
			state->addJointq(State::ACTIVEJOINT, q);
			rovin::Kinematics::solveForwardKinematics(gAssem, *state, State::LINKS_POS);
			c = clock();
		}
		renderer.updateFrame();
	}
	
	renderer._viewer.run();
	return 0;
}