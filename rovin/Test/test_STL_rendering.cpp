#include <rovin/Renderer/SimpleOSG.h>

#include "efortRobot.h"


using namespace std;
using namespace rovin::Math;
using namespace rovin::Model;
using namespace rovin::Renderer;

efortRobot gAssem;

int main()
{
	StatePtr state = gAssem.makeState();
	SimpleOSG renderer(gAssem, *state, 600, 600);
	renderer._viewer.realize();

	double c = clock();

	auto q = state->getJointq(State::STATEJOINT);
	while (1)
	{
		if (clock() - c >= 30)
		{
			state->addJointq(State::ACTIVEJOINT, q.setOnes()*0.1);
			rovin::Kinematics::solveForwardKinematics(gAssem, *state, State::LINKS_POS);
			c = clock();
		}
		renderer.updateFrame();
	}
	
	//renderer._viewer.run();
	return 0;
}