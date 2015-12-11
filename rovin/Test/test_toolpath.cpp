#include <iostream>
#include <conio.h>
#include <ctime>

#include <rovin/Math/Inertia.h>
#include <rovin/Math/Common.h>
#include <rovin/Math/Constant.h>
#include <rovin/Math/LieGroup.h>
#include <rovin/Model/Assembly.h>
#include <rovin/Model/RevoluteJoint.h>
#include <rovin/Model/MotorJoint.h>
#include <rovin/Dynamics/Kinematics.h>
#include <rovin/Dynamics/Dynamics.h>
#include <rovin/Math/Optimization.h>
#include <rovin/TrajectoryOptimization/GivenPathOptimization.h>
#include <rovin/Renderer/SimpleOSG.h>
#include <rovin/utils/Diagnostic.h>
#include <rovin/utils/fileIO.h>

using namespace std;
using namespace rovin::Math;
using namespace rovin::Model;
using namespace rovin::Renderer;
using namespace rovin::TrajectoryOptimization;

socAssemblyPtr openchain;
socAssemblyPtr effortRobot;
unsigned int dof;
VectorX q0;
VectorX qf;
VectorX dq0;
VectorX dqf;
VectorX ddq0;
VectorX ddqf;

void effortRobotModeling();
void setBoundaryValues(bool checkq0, bool checkqf, bool checkdq0, bool checkdqf, bool checkddq0, bool checkddqf);

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


int main()
{
	effortRobotModeling();
	double total_time = clock();

	GivenPathOptimization givenPath;

	givenPath.loadToolPath("toolpath_test.txt");
	Real curvTol = 500;
	givenPath.truncatePath(curvTol);



	//cout << givenPath._posTrj << endl;

	cout << "=======================================================" << endl;
	cout << "TOTAL COMPUTATION TIME WITHOUT MODELING" << endl;
	cout << clock() - total_time << endl;

	return 0;
}

void effortRobotModeling()
{
	openchain = socAssemblyPtr(new socAssembly("effortRobot"));

	shared_ptr< Link > L1 = shared_ptr< Link >(new Link("L1"));
	shared_ptr< Link > L2 = shared_ptr< Link >(new Link("L2"));
	shared_ptr< Link > L3 = shared_ptr< Link >(new Link("L3"));
	shared_ptr< Link > L4 = shared_ptr< Link >(new Link("L4"));
	shared_ptr< Link > L5 = shared_ptr< Link >(new Link("L5"));
	shared_ptr< Link > L6 = shared_ptr< Link >(new Link("L6"));
	shared_ptr< Link > L7 = shared_ptr< Link >(new Link("L7"));
	L1->addDrawingShapes(shared_ptr< Box >(new Box(0.7, 1.5, 0.7)));
	L2->addDrawingShapes(shared_ptr< Box >(new Box(0.7, 1.5, 0.7)));
	L3->addDrawingShapes(shared_ptr< Box >(new Box(0.7, 1.5, 0.7)));
	L4->addDrawingShapes(shared_ptr< Box >(new Box(0.7, 1.5, 0.7)));
	L5->addDrawingShapes(shared_ptr< Box >(new Box(0.7, 1.5, 0.7)));
	L6->addDrawingShapes(shared_ptr< Box >(new Box(0.7, 1.5, 0.7)));
	L7->addDrawingShapes(shared_ptr< Box >(new Box(0.7, 1.5, 0.7)));
	openchain->addLink(L1);
	openchain->addLink(L2);
	openchain->addLink(L3);
	openchain->addLink(L4);
	openchain->addLink(L5);
	openchain->addLink(L6);
	openchain->addLink(L7);
	VectorX len(6);
	len << 0.504, 0.170, 0.780, 0.140, 0.760, 0.125;
	openchain->addMate(shared_ptr< Joint >(new MotorJoint("J1")), "L1", "L2", SE3(Vector3(0, 0, len(0))), SE3(Vector3(0, 0, 0)));
	openchain->addMate(shared_ptr< Joint >(new MotorJoint("J2")), "L2", "L3", SE3(SO3::RotX(-PI_HALF)*SO3::RotZ(-PI_HALF), Vector3(len(1), 0, 0)), SE3(Vector3(0, 0, 0)));
	openchain->addMate(shared_ptr< Joint >(new MotorJoint("J3")), "L3", "L4", SE3(Vector3(len(2), 0, 0)), SE3(Vector3(0, 0, 0)));
	openchain->addMate(shared_ptr< Joint >(new MotorJoint("J4")), "L4", "L5", SE3(SO3::RotX(-PI_HALF), Vector3(len(3), len(4), 0)), SE3(Vector3(0, 0, 0)));
	openchain->addMate(shared_ptr< Joint >(new MotorJoint("J5")), "L5", "L6", SE3(SO3::RotX(PI_HALF), Vector3(0, 0, 0)), SE3(Vector3(0, 0, 0)));
	openchain->addMate(shared_ptr< Joint >(new MotorJoint("J6")), "L6", "L7", SE3(SO3::RotX(-PI_HALF), Vector3(0, 0, 0)), SE3(Vector3(0, 0, 0)));

	dof = 6;

	// set inertia

	MatrixX par(10, 6);
	par << 1.0000, -0.2726, 0.4164, 0.0379, 0.0379, 0.0379
		, 1.0000, 72.0862, 6.3893, 0.3574, -0.1061, -0.0759
		, 1.0000, -0.0000, 23.4120, 0.0124, 0.1181, 0.0256
		, 1.0000, 1.0000, -0.4166, -1.0914, -0.0124, 0.1181
		, 1.0000, 14.9404, 0.1093, 1.6195, 0.8195, 0.8034
		, 1.0000, -14.8427, -0.0116, 0.1966, 0.4121, 0.8312
		, 0.0478, 9.4016, 1.7068, -0.3880, 0.8152, 0.5500
		, 1.0000, -1.2929, 5.4276, -0.1850, 0.1343, -0.0108
		, 1.0000, 0.0320, 0.6453, 0.1541, -0.1275, 0.0534
		, 1.0000, -4.4974, -0.1622, -0.1995, -0.0834, 0.0317;

	VectorX m = par.row(0);
	MatrixX mx = par.block(1, 0, 3, 6);
	MatrixX inr_info = par.block(4, 0, 6, 6).transpose();
	Matrix3 I;
	vector<Inertia, Eigen::aligned_allocator<Inertia>> G(dof);
	Vector3 p;
	for (unsigned int i = 0; i < dof; i++)
	{
		I(0, 0) = inr_info(i, 0);
		I(1, 1) = inr_info(i, 1);
		I(2, 2) = inr_info(i, 2);
		I(0, 1) = I(1, 0) = inr_info(i, 3);
		I(0, 2) = I(2, 0) = inr_info(i, 4);
		I(1, 2) = I(2, 1) = inr_info(i, 5);
		p = -mx.col(i);
		G[i] = Inertia(I, p, m(i));
		//cout << i << endl << G[i] << endl;
	}
	for (unsigned int i = 0; i < openchain->getMateList().size(); i++)
		openchain->getLinkPtr("L" + to_string(i + 2))->setInertia(G[i]);

	// set motor parameters
	VectorX L(dof);
	VectorX R(dof);
	VectorX kt(dof);
	VectorX kb(dof);
	VectorX J(dof);
	VectorX gearRatio(dof);
	L << 3.5, 3.5, 5.2, 8, 8, 8;
	L *= 1e-3;

	R << 0.58, 0.58, 0.8, 2.9, 2.9, 7.5;
	kt << 0.73, 0.73, 0.5, 0.4, 0.4, 0.39;
	kb = kt;
	J << 1.06, 1.06, 0.13, 0.044, 0.044, 0.027;
	J *= 1e-3;
	gearRatio << 147, 153, 153, 76.95, 80, 51;
	for (unsigned int i = 0; i < dof; i++)
	{
		static_pointer_cast<MotorJoint> (openchain->getJointPtrByMateIndex(i))->setInductance(L(i));
		static_pointer_cast<MotorJoint> (openchain->getJointPtrByMateIndex(i))->setResistance(R(i));
		static_pointer_cast<MotorJoint> (openchain->getJointPtrByMateIndex(i))->setMotorConstant(kt(i));
		static_pointer_cast<MotorJoint> (openchain->getJointPtrByMateIndex(i))->setBackEMFConstant(kb(i));
		static_pointer_cast<MotorJoint> (openchain->getJointPtrByMateIndex(i))->setRotorInertia(J(i));
		static_pointer_cast<MotorJoint> (openchain->getJointPtrByMateIndex(i))->setGearRatio(gearRatio(i));
	}



	// set constraints

	VectorX taumax(dof);
	VectorX taumin(dof);
	VectorX qmax(dof);
	VectorX qmin(dof);
	VectorX qdotmax(dof);
	VectorX qdotmin(dof);
	VectorX qddotmax(dof);
	VectorX qddotmin(dof);
	VectorX qdddotmax(dof);
	VectorX qdddotmin(dof);
	VectorX kv(dof);
	VectorX kc(dof);
	VectorX smax(dof);
	VectorX imax(dof);

	qmax << 175, 90, 70, 180, 135, 360;
	qmax *= PI / 180;

	qmin << 175, 100, 145, 180, 135, 360;
	qmin *= -PI / 180;
	qdotmax << 100, 80, 140, 290, 290, 440;
	qdotmax *= PI / 180;

	qddotmax = 2 * qdotmax;             // user defined
	qddotmin = -qddotmax;
	qdddotmax = 300 * VectorX::Ones(dof);          // user defined
	qdddotmin = -qdddotmax;
	taumax = 3000 * VectorX::Ones(dof);          // user defined

	kv << 105.4116, 107.5105, 8.1031, 6.5430, 11.3551, 4.2050;
	kc << 92.3012, 117.3703, 57.8389, 10.0524, 24.6819, 20.32;
	smax << 3000, 3000, 4500, 4500, 4500, 4500;
	smax *= 2 * PI / 60;    // smax = [5000, 5000, 5000, 5000, 5000, 50000] * 2 * pi / 60,    % motor maximum
	imax << 39, 39, 20, 12, 12, 7.2;

	VectorX temptau = (imax.cwiseProduct(gearRatio)).cwiseProduct(kt);
	VectorX tempvel = smax.cwiseQuotient(gearRatio);
	taumax = taumax.cwiseMax(temptau);
	qdotmax = qdotmax.cwiseMax(tempvel);
	qdotmin = -qdotmax;
	taumin = -taumax;
	for (unsigned int i = 0; i < dof; i++)
	{
		openchain->getJointPtrByMateIndex(i)->setConstDamper(kv.segment(i, 1));
		openchain->getJointPtrByMateIndex(i)->setConstFriction(kc.segment(i, 1));
		openchain->getJointPtrByMateIndex(i)->setLimitPos(qmin.segment(i, 1), qmax.segment(i, 1));
		openchain->getJointPtrByMateIndex(i)->setLimitVel(qdotmin.segment(i, 1), qdotmax.segment(i, 1));
		openchain->getJointPtrByMateIndex(i)->setLimitAcc(qddotmin.segment(i, 1), qddotmax.segment(i, 1));
		openchain->getJointPtrByMateIndex(i)->setLimitJerk(qdddotmin.segment(i, 1), qdddotmax.segment(i, 1));
		openchain->getJointPtrByMateIndex(i)->setLimitInput(taumin.segment(i, 1), taumax.segment(i, 1));
	}
	openchain->completeAssembling("L1");
}

void setBoundaryValues(bool checkq0, bool checkqf, bool checkdq0, bool checkdqf, bool checkddq0, bool checkddqf)
{
	//q0.setRandom(dof);
	//qf.setRandom(dof);
	//dq0.setRandom(dof);
	//dqf.setRandom(dof);
	//ddq0.setRandom(dof);
	//ddqf.setRandom(dof);


	q0.setZero(dof);
	qf.setOnes(dof);
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