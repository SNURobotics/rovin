#include <iostream>
#include <conio.h>
#include <ctime>

#include <rovin/Math/Inertia.h>
#include <rovin/Math/Common.h>
#include <rovin/Math/Constant.h>
#include <rovin/Math/LieGroup.h>
#include <rovin/Model/Assembly.h>
#include <rovin/Model/RevoluteJoint.h>
#include <rovin/Dynamics/Kinematics.h>
#include <rovin/Dynamics/Dynamics.h>
#include <rovin/TrajectoryOptimization/PTPoptimization.h>
#include <rovin/Renderer/SimpleOSG.h>
#include <rovin/utils/Diagnostic.h>

using namespace std;
using namespace rovin::Math;
using namespace rovin::Model;
using namespace rovin::Renderer;
using namespace rovin::TrajectoryOptimization;

socAssemblyPtr openchain;
unsigned int dof;
VectorX q0;
VectorX qf;
VectorX dq0;
VectorX dqf;
VectorX ddq0;
VectorX ddqf;
void Modeling();
void setBoundaryValues(bool checkq0, bool checkqf, bool checkdq0, bool checkdqf, bool checkddq0, bool checkddqf);
int main()
{
	Modeling();
	BSplinePointToPointOptimization BsplinePTP;
	BsplinePTP.setSOCRobotModel(openchain);
	BsplinePTP.setFinalTimeAndTimeStep(1.0, 3);
	dof = openchain->makeState()->getActiveJointDof();


	bool checkq0 = true;
	bool checkqf = true;
	bool checkdq0 = true;
	bool checkdqf = true;
	bool checkddq0 = true;
	bool checkddqf = true;

	setBoundaryValues(checkq0, checkqf, checkdq0, checkdqf, checkddq0, checkddqf);

	vector<unsigned int> optActiveJointIdx(0);
	optActiveJointIdx.push_back(0);
	optActiveJointIdx.push_back(1);
	
	BsplinePTP.setBoundaryCondition(q0, qf, dq0, dqf, ddq0, ddqf);
	BsplinePTP.setConstraintRange(true, true, true, true);
	bool useWaypoint = true;
	vector<pair<VectorX, Real>> wayPoints(2);
	if (useWaypoint)
	{
		wayPoints[0].first = 0.3*VectorX::Ones(dof);
		wayPoints[0].first(1) = 0.1;
		wayPoints[0].second = 0.3;
		wayPoints[1].first = 0.7*VectorX::Ones(dof);
		wayPoints[1].first(1) = 0.2;
		wayPoints[1].second = 0.7;
		BsplinePTP.setWayPoint(wayPoints);
	}
	
	BsplinePTP.setOptimizingJointIndex(optActiveJointIdx);

	int order = 4;
	int nMiddleCP = 4;
	Real ti = 0.3;
	BsplinePTP.setSplineCondition(order, nMiddleCP);
	
	cout << "knot = " << BsplinePTP._knot.transpose() << endl;


	BsplinePTP.generateLinearEqualityConstraint();
	BsplinePTP.generateLinearInequalityConstraint();

	MatrixX Aeq_opt = BsplinePTP.Aeq_opt;
	MatrixX Aeq_nopt = BsplinePTP.Aeq_nopt;
	MatrixX beq_opt = BsplinePTP.beq_opt;
	MatrixX beq_nopt = BsplinePTP.beq_nopt;

	MatrixX Aineq_opt = BsplinePTP.Aineq_opt;
	MatrixX bineq_opt = BsplinePTP.bineq_opt;
	MatrixX Aineq_nopt = BsplinePTP.Aineq_nopt;
	MatrixX bineq_nopt = BsplinePTP.bineq_nopt;

	cout << "Aineq = " << endl;
	cout << Aineq_opt << endl;
	cout << "bineq = " << endl;
	cout << bineq_opt << endl;
	cout << "Aineq_nopt = " << endl;
	cout << Aineq_nopt << endl;
	cout << "bineq_nopt = " << endl;
	cout << bineq_nopt << endl;

	MatrixX middleCPs;
	if (Aeq_opt.rows() > 0)
		middleCPs = -(pInv(Aeq_opt)*beq_opt).transpose();
	else
		middleCPs.setRandom(1,dof*(nMiddleCP));
	MatrixX testCP(dof, (nMiddleCP + BsplinePTP._nInitCP + BsplinePTP._nFinalCP));
	for (unsigned int i = 0; i < dof; i++)
	{
		testCP.block(i, BsplinePTP._nInitCP, 1, nMiddleCP) = middleCPs.block(0, i*nMiddleCP, 1, nMiddleCP);
		for (int i = 0; i < BsplinePTP._nInitCP; i++)
			testCP.col(i) = BsplinePTP.BoundaryCP[i];
		for (int i = 0; i < BsplinePTP._nFinalCP; i++)
			testCP.col(testCP.cols() - i - 1) = BsplinePTP.BoundaryCP[5 - i];
	}
		
	BSpline<-1, -1, -1> sp(BsplinePTP._knot, testCP);
	if (useWaypoint)
	{
		for (int k = 0; k < wayPoints.size(); k++)
		{
			cout << "waypoint = " << wayPoints[k].first.transpose() << endl;
			ti = wayPoints[k].second;
			cout << "q(" << ti << ") = " << sp(ti).transpose() << endl;
		}
	}
	
	ti = 0.0;
	cout << "q(" << ti << ") = " << sp(ti).transpose() << endl;
	cout << "real q = " << q0.transpose() << endl;
	ti = 1.0 - 1e-8;
	cout << "q(" << ti << ") = " << sp(ti).transpose() << endl;
	cout << "real q = " << qf.transpose() << endl;
	if (checkdq0 || checkdqf)
	{
		BSpline<-1, -1, -1> dsp = sp.derivative();
		ti = 0.0;
		cout << "dq(" << ti << ") = " << dsp(ti).transpose() << endl;
		cout << "real dq = " << dq0.transpose() << endl;
		ti = 1.0 - 1e-8;
		cout << "dq(" << ti << ") = " << dsp(ti).transpose() << endl;
		cout << "real dq = " << dqf.transpose() << endl;
		if (checkddq0 || checkddqf)
		{
			BSpline<-1, -1, -1> ddsp = dsp.derivative();
			ti = 0.0;
			cout << "ddq(" << ti << ") = " << ddsp(ti).transpose() << endl;
			cout << "real ddq = " << ddq0.transpose() << endl;
			ti = 1.0-1e-12;
			cout << "ddq(" << ti << ") = " << ddsp(ti).transpose() << endl;
			cout << "real ddq = " << ddqf.transpose() << endl;
		}
	}
	
	return 0;
}


void Modeling()
{
	openchain = socAssemblyPtr(new socAssembly("FourbarLinkage"));

	shared_ptr< Link > L1 = shared_ptr< Link >(new Link("L1"));
	shared_ptr< Link > L2 = shared_ptr< Link >(new Link("L2"));
	shared_ptr< Link > L3 = shared_ptr< Link >(new Link("L3"));
	//shared_ptr< Link > L4 = shared_ptr< Link >(new Link("L4"));
	//shared_ptr< Link > L5 = shared_ptr< Link >(new Link("L5"));
	//shared_ptr< Link > L6 = shared_ptr< Link >(new Link("L6"));
	L1->setVisualGeometry(shared_ptr< Box >(new Box(0.7, 1.5, 0.7)));
	L2->setVisualGeometry(shared_ptr< Box >(new Box(0.7, 1.5, 0.7)));
	L3->setVisualGeometry(shared_ptr< Box >(new Box(0.7, 1.5, 0.7)));
	//L4->setVisualGeometry(shared_ptr< Box >(new Box(0.7, 1.5, 0.7)));
	//L5->setVisualGeometry(shared_ptr< Box >(new Box(0.7, 1.5, 0.7)));
	//L6->setVisualGeometry(shared_ptr< Box >(new Box(0.7, 1.5, 0.7)));
	openchain->addLink(L1);
	openchain->addLink(L2);
	openchain->addLink(L3);
	//openchain->addLink(L4);
	//openchain->addLink(L5);
	//openchain->addLink(L6);
	openchain->addMate(shared_ptr< Joint >(new RevoluteJoint("J1")), "L1", "L2", SE3(Vector3(1, 0, 0)), SE3(Vector3(1, 0, 0)));
	openchain->addMate(shared_ptr< Joint >(new RevoluteJoint("J2")), "L2", "L3", SE3(Vector3(1, 0, 0)), SE3(Vector3(1, 0, 0)));
	//openchain->addMate(shared_ptr< Joint >(new RevoluteJoint("J3")), "L3", "L4", SE3(Vector3(1, 0, 0)), SE3(Vector3(1, 0, 0)));
	//openchain->addMate(shared_ptr< Joint >(new RevoluteJoint("J4")), "L4", "L5", SE3(Vector3(1, 0, 0)), SE3(Vector3(1, 0, 0)));
	//openchain->addMate(shared_ptr< Joint >(new RevoluteJoint("J5")), "L5", "L6", SE3(Vector3(1, 0, 0)), SE3(Vector3(1, 0, 0)));

	openchain->completeAssembling("L1");
}

void setBoundaryValues(bool checkq0, bool checkqf, bool checkdq0, bool checkdqf, bool checkddq0, bool checkddqf)
{
	q0.setRandom(dof);
	qf.setRandom(dof);
	dq0.setRandom(dof);
	dqf.setRandom(dof);
	ddq0.setRandom(dof);
	ddqf.setRandom(dof);
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