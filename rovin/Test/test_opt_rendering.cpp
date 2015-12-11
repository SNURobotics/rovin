#include <rovin/Renderer/OSG_simpleRender.h>
#include <rovin/TrajectoryOptimization/PTPoptimization.h>
#include <rovin/utils/utils.h>

#include "efortRobot.h"


using namespace std;
using namespace rovin::Math;
using namespace rovin::Model;
using namespace rovin::Renderer;
using namespace rovin::TrajectoryOptimization;
using namespace rovin::utils;

socAssemblyPtr gAssem;

VectorX q0;
VectorX qf;
VectorX dq0;
VectorX dqf;
VectorX ddq0;
VectorX ddqf;

void setBoundaryValues(bool checkq0, bool checkqf, bool checkdq0, bool checkdqf, bool checkddq0, bool checkddqf);

int main()
{
	double tf = 3.0;
	double frameRate = 50;

	int order = 4;
	int nMiddleCP = 6;

	gAssem = socAssemblyPtr(new efortRobot);
	StatePtr effortState = gAssem->makeState();


	BSplinePointToPointOptimization::ObjectiveFunctionType objfunCond = BSplinePointToPointOptimization::EnergyLoss;
	shared_ptr<BSplinePointToPointOptimization> BsplinePTP = shared_ptr<BSplinePointToPointOptimization>(new BSplinePointToPointOptimization);
	BsplinePTP->setSOCRobotModel(gAssem);

	bool checkq0 = true;
	bool checkqf = true;
	bool checkdq0 = true;
	bool checkdqf = true;
	bool checkddq0 = true;
	bool checkddqf = true;

	setBoundaryValues(checkq0, checkqf, checkdq0, checkdqf, checkddq0, checkddqf);


	BsplinePTP->setBoundaryCondition(q0, qf, dq0, dqf, ddq0, ddqf);
	BsplinePTP->setConstraintRange(true, true, true, true);

	VectorU optActiveJointIdx(3);
	optActiveJointIdx << 0, 1, 2;
	BsplinePTP->setOptimizingJointIndex(optActiveJointIdx);

	///////////////////////////////////////////////////////// varying tf
	//BsplinePTP->setFinalTimeAndTimeSpan(3.0, 100);
	BsplinePTP->setFinalTimeAndTimeSpanUsingGaussianQuadrature(tf, 25);
	BsplinePTP->setSplineCondition(order, nMiddleCP, BSplinePointToPointOptimization::KnotType::Uniform);
	BsplinePTP->run(objfunCond);
	cout << "Objective : " << BsplinePTP->_fval << endl;
	cout << "Computation time : " << BsplinePTP->_computationTime << "ms" << endl;
	


	for (int iter = 0; iter < 2; iter++)
	{
		optActiveJointIdx << 3, 4, 5;
		BsplinePTP->setInitialGuess(BsplinePTP->_noptControlPoint, BsplinePTP->_solX);
		BsplinePTP->setOptimizingJointIndex(optActiveJointIdx);

		///////////////////////////////////////////////////////// varying tf
		//BsplinePTP->setFinalTimeAndTimeSpan(3.0, 100);
		BsplinePTP->setFinalTimeAndTimeSpanUsingGaussianQuadrature(tf, 25);
		BsplinePTP->setSplineCondition(order, nMiddleCP, BSplinePointToPointOptimization::KnotType::Uniform);
		BsplinePTP->run(objfunCond, false, true);
		cout << "Objective : " << BsplinePTP->_fval << endl;
		cout << "Computation time : " << BsplinePTP->_computationTime << "ms" << endl;


		optActiveJointIdx << 0, 1, 2;
		BsplinePTP->setInitialGuess(BsplinePTP->_noptControlPoint, BsplinePTP->_solX);
		BsplinePTP->setOptimizingJointIndex(optActiveJointIdx);

		///////////////////////////////////////////////////////// varying tf
		//BsplinePTP->setFinalTimeAndTimeSpan(3.0, 100);
		BsplinePTP->setFinalTimeAndTimeSpanUsingGaussianQuadrature(tf, 25);
		BsplinePTP->setSplineCondition(order, nMiddleCP, BSplinePointToPointOptimization::KnotType::Uniform);
		BsplinePTP->run(objfunCond, false, true);
		cout << "Objective : " << BsplinePTP->_fval << endl;
		cout << "Computation time : " << BsplinePTP->_computationTime << "ms" << endl;
	}



	VectorX timeSpan(frameRate * tf);
	for (int i = 0; i < timeSpan.size(); i++)
	{
		timeSpan(i) = BsplinePTP->_tf / (Real)(timeSpan.size() - 1)*(Real)i;
	}
	timeSpan(timeSpan.size() - 1) = BsplinePTP->_tf - 1e-10;
	vector<MatrixX> valvelacctau = BsplinePTP->getJointTrj(timeSpan);
	


	StatePtr state = gAssem->makeState();
	OSG_simpleRender renderer(*gAssem, *state, 600, 600);
	renderer.getViewer().realize();


	std::shared_ptr<Line>	trajLine;
	std::shared_ptr<Points>	trajPoints;
	trajLine = std::shared_ptr<Line>(new Line());
	trajPoints = std::shared_ptr<Points>(new Points());
	trajPoints->setSize(40.0);
	renderer.addGeometry(*trajLine);
	renderer.addGeometry(*trajPoints);

	double c = clock();
	int time = 0;
	while (1)
	{
		if (clock() - c >= 1000 / frameRate)
		{
			if (time >= timeSpan.size())
			{
				renderer.removeGeometry(*trajLine);
				renderer.removeGeometry(*trajPoints);
				trajLine = std::shared_ptr<Line>(new Line());
				trajPoints = std::shared_ptr<Points>(new Points());
				renderer.addGeometry(*trajLine);
				trajPoints->setSize(40.0);
				renderer.addGeometry(*trajPoints);
				time = 0;
			}
			state->setJointq(State::ACTIVEJOINT, valvelacctau[0].col(time++));
			rovin::Kinematics::solveForwardKinematics(*gAssem, *state, State::LINKS_POS);
			trajLine->push_back(eigen2osgVec<osg::Vec3>(state->getLinkState(6)._T.getPosition()));
		//	trajPoints->push_back(eigen2osgVec<osg::Vec3>(state->getLinkState(6)._T.getPosition()));
			c = clock();
		}
		renderer.updateFrame();
	}

	renderer.getViewer().run();
	return 0;
}

void setBoundaryValues(bool checkq0, bool checkqf, bool checkdq0, bool checkdqf, bool checkddq0, bool checkddqf)
{
	int dof = 6;

	srand(time(NULL));
	q0.setRandom(dof);
	qf.setRandom(dof);
	//dq0.setRandom(dof);
	//dqf.setRandom(dof);
	//ddq0.setRandom(dof);
	//ddqf.setRandom(dof);

	//q0.setOnes(dof);
	//qf.setZero(dof);
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