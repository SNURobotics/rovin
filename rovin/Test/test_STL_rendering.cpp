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

	auto q = state->getJointq(State::STATEJOINT);
	state->setJointq(State::ACTIVEJOINT, q.setRandom());
	
	rovin::Kinematics::solveForwardKinematics(gAssem, *state, State::LINKS_POS);
	renderer.updateFrame();
	
	renderer._viewer.run();
	return 0;
}