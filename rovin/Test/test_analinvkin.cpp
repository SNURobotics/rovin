#include <iostream>
#include <ctime>

#include "efortRobot.h"
#include <rovin/Math/Common.h>

using namespace std;
using namespace rovin::Math;
using namespace rovin::Model;

socAssemblyPtr gAssem;

int main()
{
	gAssem = socAssemblyPtr(new efortRobot);

	srand(time(NULL));

	StatePtr state = gAssem->makeState();
	Vector6 q;

	BilinearInterpolation bi;
	VectorX x(8), y(4);
	x << 1.0, 2.0, 5.0, 6.0, 10.0, 11.0, 12.0, 12.1;
	y << 2.2, 3.0, 7.0, 3.4;
	bi.setX(x);
	bi.setY(y);
	vector<vector<VectorX>> f;
	for (int i = 0; i < x.size(); i++)
	{
		vector<VectorX> elem;
		for (int j = 0; j < y.size(); j++)
		{
			elem.push_back(VectorX::Random(2));
			cout << elem[j] << " ";
		}
		cout << endl;
		f.push_back(elem);
	}
	bi.setElements(f);

	cout << bi(1.5, 2.6);

	//q.setRandom();
	////q << 0.8884, -1.1471, -1.0689, -0.8095, -2.9443, 1.4384;
	//cout << "========================================================================" << endl;
	//cout << "Random posture : " << endl;
	//cout << q.transpose() << endl << endl;

	//state->setJointq(State::TARGET_JOINT::ACTIVEJOINT, q);
	//cout << "======================= ENDEFFECTOR FRAME ==============================" << endl;
	//SE3 T = rovin::Kinematics::calculateEndeffectorFrame(*gAssem, *state);
	//cout << T << endl << endl;

	//StatePtr stateTEST = gAssem->makeState();
	//cout << "==================== ANALYTIC INVERSE KINEMATICS =======================" << endl;
	//vector<VectorX> invResult = rovin::Kinematics::solveInverseKinematicsOnlyForEfort(*gAssem, T);
	//for (int i = 0; i < invResult.size(); i++)
	//{
	//	cout << invResult[i].transpose() << endl;
	//	stateTEST->setJointq(State::TARGET_JOINT::ACTIVEJOINT, invResult[i]);
	//	cout << SE3::Log(rovin::Kinematics::calculateEndeffectorFrame(*gAssem, *stateTEST)*T.inverse()).transpose() << endl << endl;
	//}
	//cout << endl;

	return 0;
}