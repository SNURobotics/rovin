#include <rovin/Math/Constant.h>
#include <rovin/Renderer/OSG_simpleRender.h>
#include <rovin/TrajectoryOptimization/PTPoptimization.h>
#include <rovin/utils/utils.h>
#include <rovin/Model/State.h>
#include <rovin/Reflexxes/Core.h>

#include <windows.h>
#include <iostream>


#include "efortRobot.h"
#include "test_functions.h"


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
std::shared_ptr<Reflexxes::ReflexxesWrapper> ReflexxSolver;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
BSplinePointToPointOptimization::ObjectiveFunctionType objfunCond;
shared_ptr<BSplinePointToPointOptimization> BsplinePTP;

bool wrist = false;

shared_ptr<OSG_simpleRender> renderer;

Inertia mI(0.0);

socAssemblyPtr gAssem;

StatePtr state;

Real pmax = 1.0, vmax = 1.0, amax = 1.0;
Real sx, sy, sz;
Real ex, ey, ez;

vector<pair<VectorX, Real>> wayPoints;

Real st, sa;
Real et, ea;

shared_ptr<Points> startpoint, endpoint;
shared_ptr<Line> startzaxis, endzaxis;
Vector3 szaxis, ezaxis;
Real spsize = 10.0, epsize = 10.0;
Real tf = 3.0;

double re_timeStep = 0.01;

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
int nMiddleCP = 5;

void setBoundaryValues(bool checkq0, bool checkqf, bool checkdq0, bool checkdqf, bool checkddq0, bool checkddqf);
unsigned long __stdcall apply(void * pParam);
unsigned long __stdcall postprocess(void * pParam);
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool waypoint1 = false;
bool waypoint2 = false;


Real w1x, w1y, w1z;
Real w2x, w2y, w2z;

Real w1t, w1a;
Real w2t, w2a;

bool timeconstraint = false;

Real w1time;
Real w2time;


shared_ptr<Points> w1point, w2point;
shared_ptr<Line> waypoint1zaxis, waypoint2zaxis;
Vector3 w1zaxis, w2zaxis;
Real w1psize = 10.0, w2psize = 10.0;












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

		sx = sy = sz = 0.0;
		ex = ey = ez = 0.0;

		st = sa = 0.0;
		et = ea = 0.0;

		w1x = w1y = w1z = 0.0;
		w2x = w2y = w2z = 0.0;

		w1t = w1a = 0.0;
		w2t = w2a = 0.0;

		newstartpoint(true);
		newendpoint(true);

		ReflexxSolver = shared_ptr<Reflexxes::ReflexxesWrapper>(new Reflexxes::ReflexxesWrapper(state->getDOF(State::STATEJOINT), re_timeStep));

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

void newwaypoint1()
{
	if (!waypoint1) return;

	if (timeconstraint)
	{
		w1zaxis = Vector3(cos(w1t)*cos(w1a), sin(w1t)*cos(w1a), sin(w1a));
		waypoint1zaxis = shared_ptr<Line>(new Line);
		waypoint1zaxis->push_back(osg::Vec3(w1x, w1y, w1z));
		waypoint1zaxis->push_back(eigen2osgVec<osg::Vec3>(Vector3(w1x, w1y, w1z) + w1zaxis*0.1));
		waypoint1zaxis->setColor(0.0f, 0.0f, 0.0f);
		renderer->addGeometry(*waypoint1zaxis);
	}

	w1point = shared_ptr<Points>(new Points);
	w1point->push_back(osg::Vec3(w1x, w1y, w1z));
	w1point->setSize(w1psize);
	w1point->setColor(0.0f, 0.0f, 0.0f);
	renderer->addGeometry(*w1point);
}

void newwaypoint2()
{
	if (!waypoint2) return;

	if (timeconstraint)
	{
		w2zaxis = Vector3(cos(w2t)*cos(w2a), sin(w2t)*cos(w2a), sin(w2a));
		waypoint2zaxis = shared_ptr<Line>(new Line);
		waypoint2zaxis->push_back(osg::Vec3(w2x, w2y, w2z));
		waypoint2zaxis->push_back(eigen2osgVec<osg::Vec3>(Vector3(w2x, w2y, w2z) + w2zaxis*0.1));
		waypoint2zaxis->setColor(0.0f, 0.0f, 0.0f);
		renderer->addGeometry(*waypoint2zaxis);
	}

	w2point = shared_ptr<Points>(new Points);
	w2point->push_back(osg::Vec3(w2x, w2y, w2z));
	w2point->setSize(w2psize);
	w2point->setColor(0.0f, 0.0f, 0.0f);
	renderer->addGeometry(*w2point);
}

unsigned long __stdcall NET_RvThr(void * pParam) {
	BOOL fSuccess;
	char chBuf[500];
	DWORD dwBytesToWrite = (DWORD)strlen(chBuf);
	DWORD cbRead;

	while (1)
	{
		for (int i = 0; i < 500; i++) chBuf[i] = 0;
		fSuccess = ReadFile(hPipe, chBuf, 500, &cbRead, NULL);
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
				else if (subcommand[0].compare("energyloss") == 0)
				{
					objfunCond = BSplinePointToPointOptimization::EnergyLoss;
				}
				else if (subcommand[0].compare("effort") == 0)
				{
					objfunCond = BSplinePointToPointOptimization::Effort;
				}


				else if (subcommand[0].compare("timec") == 0)
				{
					if (waypoint1)
					{
						if (timeconstraint)
						{
							renderer->removeGeometry(*waypoint1zaxis);
						}
						renderer->removeGeometry(*w1point);
					}
					timeconstraint = true;
					newwaypoint1();
					newwaypoint2();
				}
				else if (subcommand[0].compare("timeuc") == 0)
				{
					if (timeconstraint)
					{
						if (waypoint1)
						{
							renderer->removeGeometry(*waypoint1zaxis);
						}
						if (waypoint2)
						{
							renderer->removeGeometry(*waypoint2zaxis);
						}
					}
					timeconstraint = false;
				}


				else if (subcommand[0].compare("w1c") == 0)
				{
					if (waypoint1)
					{
						if (timeconstraint)
						{
							renderer->removeGeometry(*waypoint1zaxis);
						}
						renderer->removeGeometry(*w1point);
					}
					if (waypoint2)
					{
						if (timeconstraint)
						{
							renderer->removeGeometry(*waypoint2zaxis);
						}
						renderer->removeGeometry(*w2point);
					}
					waypoint1 = true;
					newwaypoint1();
				}
				else if (subcommand[0].compare("w1uc") == 0)
				{
					if (waypoint1)
					{
						if (timeconstraint)
						{
							renderer->removeGeometry(*waypoint1zaxis);
						}
						renderer->removeGeometry(*w1point);
					}
					waypoint1 = false;
				}
				else if (subcommand[0].compare("w1x") == 0)
				{
					w1x = stod(subcommand[1]) / 100.0 * pmax * 2 - pmax;
					if (waypoint1)
					{
						if (timeconstraint)
						{
							renderer->removeGeometry(*waypoint1zaxis);
						}
						renderer->removeGeometry(*w1point);
					}
					newwaypoint1();
				}
				else if (subcommand[0].compare("w1y") == 0)
				{
					w1y = stod(subcommand[1]) / 100.0 * pmax * 2 - pmax;
					if (waypoint1)
					{
						if (timeconstraint)
						{
							renderer->removeGeometry(*waypoint1zaxis);
						}
						renderer->removeGeometry(*w1point);
					}
					newwaypoint1();
				}
				else if (subcommand[0].compare("w1z") == 0)
				{
					w1z = stod(subcommand[1]) / 100.0 * pmax * 2;
					if (waypoint1)
					{
						if (timeconstraint)
						{
							renderer->removeGeometry(*waypoint1zaxis);
						}
						renderer->removeGeometry(*w1point);
					}
					newwaypoint1();
				}
				else if (subcommand[0].compare("w1t") == 0)
				{
					w1t = stod(subcommand[1]) / 100.0 * pmax * 2 - pmax;
					if (waypoint1)
					{
						if (timeconstraint)
						{
							renderer->removeGeometry(*waypoint1zaxis);
						}
						renderer->removeGeometry(*w1point);
					}
					newwaypoint1();
				}
				else if (subcommand[0].compare("w1a") == 0)
				{
					w1a = stod(subcommand[1]) / 100.0 * pmax * 2 - pmax;
					if (waypoint1)
					{
						if (timeconstraint)
						{
							renderer->removeGeometry(*waypoint1zaxis);
						}
						renderer->removeGeometry(*w1point);
					}
					newwaypoint1();
				}
				else if (subcommand[0].compare("w1time") == 0)
				{
					w1time = stod(subcommand[1]) / 100.0;
				}


				else if (subcommand[0].compare("w2c") == 0)
				{
					if (waypoint2)
					{
						if (timeconstraint)
						{
							renderer->removeGeometry(*waypoint2zaxis);
						}
						renderer->removeGeometry(*w2point);
					}
					waypoint2 = true;
					newwaypoint2();
				}
				else if (subcommand[0].compare("w2uc") == 0)
				{
					if (waypoint2)
					{
						if (timeconstraint)
						{
							renderer->removeGeometry(*waypoint2zaxis);
						}
						renderer->removeGeometry(*w2point);
					}
					waypoint2 = false;
				}
				else if (subcommand[0].compare("w2x") == 0)
				{
					w2x = stod(subcommand[1]) / 100.0 * pmax * 2 - pmax;
					if (waypoint2)
					{
						if (timeconstraint)
						{
							renderer->removeGeometry(*waypoint2zaxis);
						}
						renderer->removeGeometry(*w2point);
					}
					newwaypoint2();
				}
				else if (subcommand[0].compare("w2y") == 0)
				{
					w2y = stod(subcommand[1]) / 100.0 * pmax * 2 - pmax;
					if (waypoint2)
					{
						if (timeconstraint)
						{
							renderer->removeGeometry(*waypoint2zaxis);
						}
						renderer->removeGeometry(*w2point);
					}
					newwaypoint2();
				}
				else if (subcommand[0].compare("w2z") == 0)
				{
					w2z = stod(subcommand[1]) / 100.0 * pmax * 2;
					if (waypoint2)
					{
						if (timeconstraint)
						{
							renderer->removeGeometry(*waypoint2zaxis);
						}
						renderer->removeGeometry(*w2point);
					}
					newwaypoint2();
				}
				else if (subcommand[0].compare("w2t") == 0)
				{
					w2t = stod(subcommand[1]) / 100.0 * pmax * 2 - pmax;
					if (waypoint2)
					{
						if (timeconstraint)
						{
							renderer->removeGeometry(*waypoint2zaxis);
						}
						renderer->removeGeometry(*w2point);
					}
					newwaypoint2();
				}
				else if (subcommand[0].compare("w2a") == 0)
				{
					w2a = stod(subcommand[1]) / 100.0 * pmax * 2 - pmax;
					if (waypoint2)
					{
						if (timeconstraint)
						{
							renderer->removeGeometry(*waypoint2zaxis);
						}
						renderer->removeGeometry(*w2point);
					}
					newwaypoint2();
				}
				else if (subcommand[0].compare("w2time") == 0)
				{
					w2time = stod(subcommand[1]) / 100.0;
				}

				else if (subcommand[0].compare("ew") == 0)
				{
					double m = stod(subcommand[1]) / 100.0 * 5.0;
					Inertia mm(m);
					mm.changeFrame(gAssem->_socLink[6]._M * TOOLTIP);
					gAssem->_socLink[6]._G -= mI;
					gAssem->_socLink[6]._G += mm;
					mI = mm;
				}

				else if (subcommand[0].compare("wrist") == 0)
				{
					wrist = true;
				}
				else if (subcommand[0].compare("uwrist") == 0)
				{
					wrist = false;
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

	BsplinePTP->_waypointPositionExist = false;
	wayPoints = vector<pair<VectorX, Real>>();

	if (timeconstraint)
	{
		if (waypoint1)
		{
			VectorX w1;
			z = w1zaxis;
			if (w1zaxis(0) != 0) x << -z(1), z(0), 0;
			else if (w1zaxis(1) != 0) x << 0, -z(2), z(1);
			else if (w1zaxis(2) != 0) x << z(2), 0, -z(0);
			y = z.cross(x);
			R.col(0) = x; R.col(1) = y; R.col(2) = z;
			goalT = SE3(Vector3(w1x, w1y, w1z)) * SE3(SO3(R));
			inversek = rovin::Kinematics::solveInverseKinematicsOnlyForEfort(*gAssem, goalT*TOOLTIP.inverse());
			min = RealMax;
			flag = false;
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
						w1 = inversek[i];
					}
				}
			}
			if (flag == false)
			{
				cout << "ERROR - WAYPOINT1 IS NOT AVAILABLE!!" << endl;
				solving = false;
				return 0;
			}
			wayPoints.push_back(pair<VectorX, Real>(w1, w1time));
		}
		if (waypoint1 || waypoint2)
		{
			BsplinePTP->setWayPoint(wayPoints);
		}
	}
	else
	{
		if (waypoint1 || waypoint2)
		{
			vector<Vector3> waypointposition;

			if (waypoint1)
			{
				waypointposition.push_back(Vector3(w1x, w1y, w1z));
			}
			if (waypoint2)
			{
				waypointposition.push_back(Vector3(w2x, w2y, w2z));
			}

			BsplinePTP->setWayPointOnlyPosition(waypointposition);
		}
	}

	BsplinePTP->setBoundaryCondition(q0, qf, dq0, dqf, ddq0, ddqf);
	BsplinePTP->setConstraintRange(true, true, true, true);

	VectorU optActiveJointIdx;
	if (wrist)
	{
		optActiveJointIdx.setZero(6);
		optActiveJointIdx << 0, 1, 2, 3, 4, 5;
	}
	else
	{
		optActiveJointIdx.setZero(3);
		optActiveJointIdx << 0, 1, 2;
	}
	BsplinePTP->setOptimizingJointIndex(optActiveJointIdx);

	cout << "OPTIMIZATION START" << endl;
	order = 4;
	nMiddleCP = 4;
	if ((waypoint1 || waypoint2) && !timeconstraint)
	{
		nMiddleCP = 5;
		BsplinePTP->setFinalTimeAndTimeSpanUsingGaussianQuadrature(tf, 100);
	}
	else
	{
		BsplinePTP->setFinalTimeAndTimeSpanUsingGaussianQuadrature(tf, 25);
	}
	BsplinePTP->setSplineCondition(order, nMiddleCP, BSplinePointToPointOptimization::KnotType::Uniform);
	if (BsplinePTP->run(objfunCond, (waypoint1 || waypoint2) && timeconstraint).size() == 0)
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
	cout << " Objective Funcion = " << BsplinePTP->_objectiveFunc->func(BsplinePTP->_solX) << endl;
	cout << "=====================================================" << endl << endl;

	///////////////////////////////REFLEXXES////////////////////////////////////////////////////////////////////////

	if ((waypoint1 || waypoint2) && timeconstraint)
	{
		MatrixX position(6, 2 + wayPoints.size());
		position.col(0) = q0;
		for (unsigned int i = 0; i < wayPoints.size();i++)
		{
			position.col(i + 1) = wayPoints[i].first;
		}
		position.col(wayPoints.size() + 1) = qf;
		ReflexxSolver->setWayPointsPos(position);
	}
	else if((waypoint1 || waypoint2) && !timeconstraint)
	{
		vector<VectorX> pp;
		pp.push_back(q0);
		StatePtr tempState = gAssem->makeState();
		if (waypoint1)
		{
			VectorX tempX;
			double min = RealMax;
			for (int i = 0; i < ntrajectory; i++)
			{
				tempState->setJointq(State::TARGET_JOINT::ACTIVEJOINT, trajectory.col(i));
				double tempDistance = (Vector3(w1x, w1y, w1z) - (rovin::Kinematics::calculateEndeffectorFrame(*gAssem, *tempState)*TOOLTIP).getPosition()).norm();
				if (min > tempDistance)
				{
					min = tempDistance;
					tempX = trajectory.col(i);
				}
			}
			pp.push_back(tempX);
		}
		if (waypoint2)
		{
			VectorX tempX;
			double min = RealMax;
			for (int i = 0; i < ntrajectory; i++)
			{
				tempState->setJointq(State::TARGET_JOINT::ACTIVEJOINT, trajectory.col(i));
				double tempDistance = (Vector3(w2x, w2y, w2z) - (rovin::Kinematics::calculateEndeffectorFrame(*gAssem, *tempState)*TOOLTIP).getPosition()).norm();
				if (min > tempDistance)
				{
					min = tempDistance;
					tempX = trajectory.col(i);
				}
			}
			pp.push_back(tempX);
		}
		pp.push_back(qf);

		MatrixX position(6, pp.size());
		for (unsigned int i = 0; i < pp.size();i++)
		{
			position.col(i) = pp[i];
		}
		ReflexxSolver->setWayPointsPos(position);
	}
	else
	{
		MatrixX position(6, 2);
		position.col(0) = q0;
		position.col(1) = qf;
		ReflexxSolver->setWayPointsPos(position);
	}

	for (unsigned int i = 0; i < state->getDOF(State::STATEJOINT); i++)
	{
		ReflexxSolver->_dqLim[i] = gAssem->getJointPtrByMateIndex(i)->getLimitVelUpper().operator[](0);
		ReflexxSolver->_ddqLim[i] = gAssem->getJointPtrByMateIndex(i)->getLimitAccUpper().operator[](0);
	}
	double re_c = clock();
	MatrixX traj = ReflexxSolver->solve();
	double re_ctime = clock() - re_c;
	auto torque = calcTorque(*gAssem, ReflexxSolver->getResultPos(), ReflexxSolver->getResultVel(), ReflexxSolver->getResultAcc());

	Real Cost = 0.0;
	MatrixX val(6, traj.cols());
	for (int i = 0; i < traj.cols(); i++)
	{
		efort_state->setJointq(State::ACTIVEJOINT, traj.col(i));
		reflexxes->push_back(eigen2osgVec<osg::Vec3>((rovin::Kinematics::calculateEndeffectorFrame(*gAssem, *efort_state) * TOOLTIP).getPosition()));
	}
	renderer->addGeometry(*reflexxes);

	if (objfunCond == BSplinePointToPointOptimization::Effort)
	{
		Cost = calcEffort(torque).sum()*re_timeStep;
	}
	else
	{
		Cost = calcEnergy(*gAssem, ReflexxSolver->getResultVel(), ReflexxSolver->getResultAcc(), torque).sum()*re_timeStep;
	}
	cout << "===== REFLEXXES ====================================" << endl;
	cout << " Tf = " << (traj.cols() - 1) * re_timeStep << endl;
	cout << " Computation Time = " << re_ctime << endl;
	cout << " Objective Funcion = " << Cost << endl;
	cout << "=====================================================" << endl << endl;
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	firstLineDrawing = false;

	return 0;
}