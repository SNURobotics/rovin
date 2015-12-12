#include <rovin/Math/Constant.h>
#include <rovin/Renderer/OSG_simpleRender.h>
#include <rovin/TrajectoryOptimization/PTPoptimization.h>
#include <rovin/utils/utils.h>
#include <rovin/Model/State.h>
#include <rovin/Reflexxes/Core.h>

#include <windows.h>
#include <iostream>


#include "efortRobot.h"


using namespace std;
using namespace rovin::Math;
using namespace rovin::Model;
using namespace rovin::Renderer;
using namespace rovin::TrajectoryOptimization;
using namespace rovin::utils;

unsigned long __stdcall NET_RvThr(void * pParam);
DWORD WINAPI ThreadProc();
HANDLE hPipe;
BOOL Finished;

SE3 TOOLTIP(Vector3(0, 0, 0.12));

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
BSplinePointToPointOptimization::ObjectiveFunctionType objfunCond;
shared_ptr<BSplinePointToPointOptimization> BsplinePTP;

shared_ptr<OSG_simpleRender> renderer;

socAssemblyPtr gAssem;

StatePtr state;

Real pmax = 1.0, vmax = 1.0, amax = 1.0;
Real sx, sy, sz, sdx, sdy, sdz, sddx, sddy, sddz;
Real ex, ey, ez, edx, edy, edz, eddx, eddy, eddz;

Real st, sa;
Real et, ea;

shared_ptr<Points> startpoint, endpoint;
shared_ptr<Line> startzaxis, endzaxis;
Vector3 szaxis, ezaxis;
Real spsize = 10.0, epsize = 10.0;
Real tf = 3.0;

bool firstLineDrawing = true;
shared_ptr<Line> ours, reflexxes, linear;

VectorX q0;
VectorX qf;
VectorX dq0;
VectorX dqf;
VectorX ddq0;
VectorX ddqf;

vector<MatrixX> valvelacctau;

int ntrajectory = 0;
int tt = 0;
MatrixX trajectory;
bool usingtrajectory = false;
bool solving = false;

double frameRate = 50;

int order = 4;
int nMiddleCP = 4;

void setBoundaryValues(bool checkq0, bool checkqf, bool checkdq0, bool checkdqf, bool checkddq0, bool checkddqf);
unsigned long __stdcall apply(void * pParam);
unsigned long __stdcall postprocess(void * pParam);
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void newstartpoint(bool);
void newendpoint(bool);

int main(int argc, char *argv[])
{
	LPTSTR lpszPipename = TEXT("\\\\.\\pipe\\NamePipeGoogolTech");

	//Thread Init Data
	HANDLE hThread = NULL;

	BOOL Write_St = TRUE;

	Finished = FALSE;

	hPipe = CreateFile(lpszPipename, GENERIC_READ, 0, NULL, OPEN_EXISTING, FILE_FLAG_OVERLAPPED, NULL);


	if (hPipe == NULL || hPipe == INVALID_HANDLE_VALUE)
	{
		printf("Please execute the control panel  - (error %d)\n", GetLastError());

	}
	else
	{

		gAssem = socAssemblyPtr(new efortRobot);

		objfunCond = BSplinePointToPointOptimization::Effort;
		BsplinePTP = shared_ptr<BSplinePointToPointOptimization>(new BSplinePointToPointOptimization);
		BsplinePTP->setSOCRobotModel(gAssem);
		setBoundaryValues(true, true, true, true, true, true);

		state = gAssem->makeState();
		renderer = shared_ptr<OSG_simpleRender>(new OSG_simpleRender(*gAssem, *state, 600, 600));
		renderer->getViewer().realize();

		state->setJointq(State::TARGET_JOINT::ACTIVEJOINT, Vector6::Zero());
		rovin::Kinematics::solveForwardKinematics(*gAssem, *state, State::LINKS_POS);

		sx = sy = sz = sdx = sdy = sdz = sddx = sddy = sddz = 0.0;
		ex = ey = ez = edx = edy = edz = eddx = eddy = eddz = 0.0;

		st = sa = 0.0;
		et = ea = 0.0;

		newstartpoint(true);
		newendpoint(true);

		hThread = CreateThread(NULL, 0, &NET_RvThr, NULL, 0, NULL);

		double c = clock();
		while (1)
		{
			if (clock() - c >= 1000 / frameRate)
			{
				if (ntrajectory != 0 && !usingtrajectory)
				{
					if (tt >= ntrajectory)
					{
						tt = 0;
					}
					state->setJointq(State::TARGET_JOINT::ACTIVEJOINT, trajectory.col(tt++));
					rovin::Kinematics::solveForwardKinematics(*gAssem, *state, State::LINKS_POS);
					//trajLine->push_back(eigen2osgVec<osg::Vec3>(state->getLinkState(6)._T.getPosition()));
					//	trajPoints->push_back(eigen2osgVec<osg::Vec3>(state->getLinkState(6)._T.getPosition()));
				}
				c = clock();
			}
			renderer->updateFrame();
		}

		CloseHandle(hPipe);
		Finished = TRUE;
	}

	getchar();
}

void newstartpoint(bool first = false)
{
	if (!first)
	{
		renderer->removeGeometry(*startpoint);
		renderer->removeGeometry(*startzaxis);
	}

	szaxis = Vector3(cos(st)*cos(sa), sin(st)*cos(sa), sin(sa));
	startzaxis = shared_ptr<Line>(new Line);
	startzaxis->push_back(osg::Vec3(sx, sy, sz));
	startzaxis->push_back(eigen2osgVec<osg::Vec3>(Vector3(sx, sy, sz) + szaxis*0.1));
	startzaxis->setColor(0.0f, 0.0f, 1.0f);
	renderer->addGeometry(*startzaxis);

	startpoint = shared_ptr<Points>(new Points);
	startpoint->push_back(osg::Vec3(sx, sy, sz));
	startpoint->setSize(spsize);
	startpoint->setColor(0.0f, 0.0f, 1.0f);
	renderer->addGeometry(*startpoint);
}

void newendpoint(bool first = false)
{
	if (!first)
	{
		renderer->removeGeometry(*endpoint);
		renderer->removeGeometry(*endzaxis);
	}

	ezaxis = Vector3(cos(et)*cos(ea), sin(et)*cos(ea), sin(ea));
	endzaxis = shared_ptr<Line>(new Line);
	endzaxis->push_back(osg::Vec3(ex, ey, ez));
	endzaxis->push_back(eigen2osgVec<osg::Vec3>(Vector3(ex, ey, ez) + ezaxis*0.1));
	endzaxis->setColor(1.0f, 0.0f, 0.0f);
	renderer->addGeometry(*endzaxis);

	endpoint = shared_ptr<Points>(new Points);
	endpoint->push_back(osg::Vec3(ex, ey, ez));
	endpoint->setSize(epsize);
	endpoint->setColor(1.0f, 0.0f, 0.0f);
	renderer->addGeometry(*endpoint);
}

unsigned long __stdcall NET_RvThr(void * pParam) {
	BOOL fSuccess;
	char chBuf[100];
	DWORD dwBytesToWrite = (DWORD)strlen(chBuf);
	DWORD cbRead;

	while (1)
	{
		for (int i = 0; i < 100; i++) chBuf[i] = 0;
		fSuccess = ReadFile(hPipe, chBuf, 100, &cbRead, NULL);
		if (fSuccess)
		{
			int ix;
			string command(chBuf);
			while ((ix = command.find('=')) != std::string::npos)
			{
				string scommand = command.substr(0, ix);
				command.erase(0, ix + 1);

				cout << scommand << endl;
				vector<string> subcommand;
				int count = 0;
				int idx = 0;
				while ((idx = scommand.find(' ')) != std::string::npos)
				{
					subcommand.push_back(scommand.substr(0, idx));
					scommand.erase(0, idx + 1);
				}
				subcommand.push_back(scommand);

				if (subcommand[0].compare("sx") == 0)
				{
					sx = stod(subcommand[1]) / 100.0 * pmax * 2 - pmax;
					newstartpoint();
				}
				else if (subcommand[0].compare("sy") == 0)
				{
					sy = stod(subcommand[1]) / 100.0 * pmax * 2 - pmax;
					newstartpoint();
				}
				else if (subcommand[0].compare("sz") == 0)
				{
					sz = stod(subcommand[1]) / 100.0 * pmax * 2;
					newstartpoint();
				}
				else if (subcommand[0].compare("ex") == 0)
				{
					ex = stod(subcommand[1]) / 100.0 * pmax * 2 - pmax;
					newendpoint();
				}
				else if (subcommand[0].compare("ey") == 0)
				{
					ey = stod(subcommand[1]) / 100.0 * pmax * 2 - pmax;
					newendpoint();
				}
				else if (subcommand[0].compare("ez") == 0)
				{
					ez = stod(subcommand[1]) / 100.0 * pmax * 2;
					newendpoint();
				}
				else if (subcommand[0].compare("tf") == 0)
				{
					tf = stod(subcommand[1]) / 10.0;
					newendpoint();
				}
				else if (subcommand[0].compare("apply") == 0)
				{
					HANDLE hThread = NULL;
					hThread = CreateThread(NULL, 0, &apply, NULL, 0, NULL);
				}
				else if (subcommand[0].compare("st") == 0)
				{
					st = stod(subcommand[1]) / 100.0 * 2 * Math::PI - Math::PI;
					newstartpoint();
				}
				else if (subcommand[0].compare("sa") == 0)
				{
					sa = stod(subcommand[1]) / 100.0 * Math::PI - Math::PI / 2;
					newstartpoint();
				}
				else if (subcommand[0].compare("et") == 0)
				{
					et = stod(subcommand[1]) / 100.0 * 2 * Math::PI - Math::PI;
					newendpoint();
				}
				else if (subcommand[0].compare("ea") == 0)
				{
					ea = stod(subcommand[1]) / 100.0 * Math::PI - Math::PI / 2;
					newendpoint();
				}
			}
		}
		if (!fSuccess && GetLastError() != ERROR_MORE_DATA)
		{
			if (Finished)
				break;
		}
	}

	return 0;
}

void setBoundaryValues(bool checkq0, bool checkqf, bool checkdq0, bool checkdqf, bool checkddq0, bool checkddqf)
{
	int dof = 6;

	srand(time(NULL));
	//q0.setRandom(dof);
	//qf.setRandom(dof);
	//dq0.setRandom(dof);
	//dqf.setRandom(dof);
	//ddq0.setRandom(dof);
	//ddqf.setRandom(dof);

	qf.setZero(dof);
	q0.setZero(dof);
	dq0.setZero(dof);
	dqf.setZero(dof);
	ddq0.setZero(dof);
	ddqf.setZero(dof);

	if (!checkq0)
	{
		q0.resize(0); dq0.resize(0); ddq0.resize(0);
	}
	if (!checkqf)
	{
		qf.resize(0); dqf.resize(0); ddqf.resize(0);
	}
	if (!checkdq0)
	{
		dq0.resize(0); ddq0.resize(0);
	}
	if (!checkdqf)
	{
		dqf.resize(0); ddqf.resize(0);
	}
	if (!checkddq0)
		ddq0.resize(0);
	if (!checkddqf)
		ddqf.resize(0);
}

VectorX effortTrj(const MatrixX& jointTorqueTrj)
{
	VectorX effortTrj(jointTorqueTrj.cols());
	for (int i = 0; i < jointTorqueTrj.cols(); i++)
		effortTrj(i) = jointTorqueTrj.col(i).squaredNorm();
	return effortTrj;
};

VectorX energyConsumptionTrj(const socAssembly& socAssem, const MatrixX& jointVelTrj, const MatrixX& jointAccTrj, const MatrixX& jointTorqueTrj)
{
	shared_ptr<MotorJoint> tempJointPtr;
	Real voltage;
	Real current;
	VectorX energyConsumptionTrj(jointTorqueTrj.cols());
	energyConsumptionTrj.setZero();
	for (int i = 0; i < jointTorqueTrj.cols(); i++)
	{
		for (unsigned int j = 0; j < socAssem.getMateList().size(); j++)
		{
			tempJointPtr = static_pointer_cast<MotorJoint>(socAssem.getJointPtrByMateIndex(j));
			current = 1.0 / (tempJointPtr->getMotorConstant() * tempJointPtr->getGearRatio()) * jointTorqueTrj(j, i)
				+ tempJointPtr->getRotorInertia() * tempJointPtr->getGearRatio() / tempJointPtr->getMotorConstant() * jointAccTrj(j, i);
			voltage = current * tempJointPtr->getResistance() + tempJointPtr->getBackEMFConstant() * tempJointPtr->getGearRatio() * jointVelTrj(j, i);
			energyConsumptionTrj(i) += max(current * voltage, 0.0);
		}
	}
	return energyConsumptionTrj;
}

unsigned long __stdcall apply(void * pParam)
{
	unsigned int i;

	while (solving);
	solving = true;

	cout << "APPLY BUTTON CLICKED" << endl;

	Matrix3 R;
	Vector3 x, y, z;
	SE3 goalT;
	vector<VectorX> inversek;

	z = szaxis;
	if (szaxis(0) != 0) x << -z(1), z(0), 0;
	else if (szaxis(1) != 0) x << 0, -z(2), z(1);
	else if (szaxis(2) != 0) x << z(2), 0, -z(0);
	y = z.cross(x);
	R.col(0) = x; R.col(1) = y; R.col(2) = z;
	goalT = SE3(Vector3(sx, sy, sz)) * SE3(SO3(R));
	inversek = rovin::Kinematics::solveInverseKinematicsOnlyForEfort(*gAssem, goalT*TOOLTIP.inverse());
	for (i = 0; i < inversek.size(); i++)
	{
		int j;
		for (j = 0; j < 6; j++)
		{
			if (gAssem->getJointPtrByMateIndex(j)->getLimitPosLower()(0) <= inversek[i](j) && inversek[i](j) <= gAssem->getJointPtrByMateIndex(j)->getLimitPosUpper()(0));
			else break;
		}
		if (j == 6)
		{
			q0 = inversek[i];
			break;
		}
	}
	if (i == inversek.size())
	{
		cout << "ERROR - START POINT IS NOT AVAILABLE!!" << endl;
		solving = false;
		return 0;
	}

	z = ezaxis;
	if (ezaxis(0) != 0) x << -z(1), z(0), 0;
	else if (ezaxis(1) != 0) x << 0, -z(2), z(1);
	else if (ezaxis(2) != 0) x << z(2), 0, -z(0);
	y = z.cross(x);
	R.col(0) = x; R.col(1) = y; R.col(2) = z;
	goalT = SE3(Vector3(ex, ey, ez)) * SE3(SO3(R));
	inversek = rovin::Kinematics::solveInverseKinematicsOnlyForEfort(*gAssem, goalT*TOOLTIP.inverse());
	Real sum, min = RealMax;
	bool flag = false;
	for (i = 0; i < inversek.size(); i++)
	{
		int j;
		for (j = 0; j < 6; j++)
		{
			if (gAssem->getJointPtrByMateIndex(j)->getLimitPosLower()(0) <= inversek[i](j) && inversek[i](j) <= gAssem->getJointPtrByMateIndex(j)->getLimitPosUpper()(0));
			else break;
		}
		if (j == 6)
		{
			flag = true;
			sum = (inversek[i] - q0).squaredNorm();
			if (min > sum)
			{
				min = sum;
				qf = inversek[i];
			}
		}
	}
	if (flag == false)
	{
		cout << "ERROR - END POINT IS NOT AVAILABLE!!" << endl;
		solving = false;
		return 0;
	}

	BsplinePTP->setBoundaryCondition(q0, qf, dq0, dqf, ddq0, ddqf);
	BsplinePTP->setConstraintRange(true, true, true, true);

	VectorU optActiveJointIdx(3);
	optActiveJointIdx << 0, 1, 2;
	BsplinePTP->setOptimizingJointIndex(optActiveJointIdx);

	cout << "OPTIMIZATION START" << endl;
	BsplinePTP->setFinalTimeAndTimeSpanUsingGaussianQuadrature(tf, 25);
	BsplinePTP->setSplineCondition(order, nMiddleCP, BSplinePointToPointOptimization::KnotType::Uniform);
	if (BsplinePTP->run(objfunCond).size() == 0)
	{
		cout << "ERROR - CANNOT FIND FEASIBLE SOLUTION!!" << endl;
		solving = false;
		return 0;
	}
	cout << "OPTIMIZATION END" << endl;
	//cout << BsplinePTP->_fval << endl;

	VectorX timeSpan(frameRate * tf);
	for (int i = 0; i < timeSpan.size(); i++)
	{
		timeSpan(i) = BsplinePTP->_tf / (Real)(timeSpan.size() - 1)*(Real)i;
	}
	timeSpan(timeSpan.size() - 1) = BsplinePTP->_tf - 1e-10;
	valvelacctau = BsplinePTP->getJointTrj(timeSpan);

	while (usingtrajectory);
	usingtrajectory = true;
	ntrajectory = timeSpan.size();
	trajectory.resize(6, ntrajectory);
	for (int i = 0; i < ntrajectory; i++)
	{
		trajectory.col(i) = valvelacctau[0].col(i);
	}

	HANDLE hThread = NULL;
	hThread = CreateThread(NULL, 0, &postprocess, NULL, 0, NULL);

	tt = 0;
	usingtrajectory = false;
	solving = false;

	return 0;
}

unsigned long __stdcall postprocess(void * pParam)
{
	if (!firstLineDrawing)
	{
		renderer->removeGeometry(*ours);
		renderer->removeGeometry(*reflexxes);
		//renderer->removeGeometry(*linear);
	}

	ours = shared_ptr<Line>(new Line);
	reflexxes = shared_ptr<Line>(new Line);
	//linear = shared_ptr<Line>(new Line);

	StatePtr efort_state = gAssem->makeState();
	for (int i = 0; i < ntrajectory; i++)
	{
		efort_state->setJointq(State::TARGET_JOINT::ACTIVEJOINT, trajectory.col(i));
		rovin::Kinematics::solveForwardKinematics(*gAssem, *efort_state, State::LINKS_POS);
		ours->push_back(eigen2osgVec<osg::Vec3>((rovin::Kinematics::calculateEndeffectorFrame(*gAssem, *efort_state) * TOOLTIP).getPosition()));
	}
	renderer->addGeometry(*ours);

	cout << "===== OUR ALGORITHM =================================" << endl;
	cout << " Tf = " << BsplinePTP->_tf << endl;
	cout << " Computation Time = " << BsplinePTP->_computationTime << endl;
	cout << " Objective Funcion = " << BsplinePTP->_fval << endl;
	cout << "=====================================================" << endl << endl;

	///////////////////////////////REFLEXXES////////////////////////////////////////////////////////////////////////
	const double re_timeStep = 0.01;

	Reflexxes::ReflexxesWrapper		ReflexxSolver(state->getDOF(State::STATEJOINT), re_timeStep);
	ReflexxSolver._q0 = q0;
	ReflexxSolver._qf = qf;
	for (unsigned int i = 0; i < state->getDOF(State::STATEJOINT); i++)
	{
		ReflexxSolver._dqLim[i] = gAssem->getJointPtrByMateIndex(i)->getLimitVelUpper().operator[](0);
		ReflexxSolver._ddqLim[i] = gAssem->getJointPtrByMateIndex(i)->getLimitAccUpper().operator[](0);
	}
	double re_c = clock();
	vector<MatrixX> traj = ReflexxSolver.solve2();
	double re_ctime = clock() - re_c;

	Real Cost = 0.0;
	MatrixX val(6, traj[0].cols());
	if (objfunCond == BSplinePointToPointOptimization::Effort)
	{
		for (int i = 0; i < traj[0].cols(); i++)
		{
			efort_state->setJointq(State::ACTIVEJOINT, traj[0].col(i));
			efort_state->setJointqdot(State::ACTIVEJOINT, traj[1].col(i));
			efort_state->setJointqddot(State::ACTIVEJOINT, traj[2].col(i));
			rovin::Dynamics::solveInverseDynamics(*gAssem, *efort_state);
			val.col(i) = efort_state->getJointTorque(State::ACTIVEJOINT);
			reflexxes->push_back(eigen2osgVec<osg::Vec3>((rovin::Kinematics::calculateEndeffectorFrame(*gAssem, *efort_state) * TOOLTIP).getPosition()));
		}
		Cost = effortTrj(val).sum() * re_timeStep;
	}
	renderer->addGeometry(*reflexxes);
	cout << "===== REFLEXXES =================================" << endl;
	cout << " Tf = " << (traj[0].cols() - 1) * re_timeStep << endl;
	cout << " Computation Time = " << re_ctime << endl;
	cout << " Objective Funcion = " << Cost << endl;
	cout << "=====================================================" << endl << endl;
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	firstLineDrawing = false;

	return 0;
}