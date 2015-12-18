/**
*	\file	efortRobot.h
*	\date	2015.12.08
*	\author	Jisoo Hong (jshong@robotics.snu.ac.kr)
*	\brief	Assembly class for modeling effort robot.
*/

#pragma once
#include <vector>
#include <string>
#include <rovin/Model/Assembly.h>
#include <rovin/Model/RevoluteJoint.h>
#include <rovin/Dynamics/Kinematics.h>
#include <rovin/Dynamics/Dynamics.h>
#include <rovin/utils/Diagnostic.h>
#include <rovin/Model/MotorJoint.h>
#include <rovin/Dynamics/Kinematics.h>

using namespace std;
using namespace rovin;
using namespace rovin::Math;
using namespace rovin::Model;

class efortRobot : public socAssembly
{
public:
	efortRobot()
		:socAssembly("Effort Robot")
	{
		_links.resize(7);
		_joints.resize(6);
		assembleModel();
		addMeshFiles();
	}

	void assembleModel()
	{
		for (unsigned int i = 0; i < _links.size(); i++)
		{
			_links[i] = shared_ptr< Link >(new Link(string("L") + std::to_string(i)));
			this->addLink(_links[i]);
		}

		for (unsigned int i = 0; i < _joints.size(); i++)
		{
			_joints[i] = shared_ptr< Joint >(new MotorJoint(string("J") + std::to_string(i)));
		}

		VectorX len(6);
		len << 0.504, 0.170, 0.780, 0.140, 0.760, 0.125;
		this->addMate(_joints[0], "L0", "L1", SE3(Vector3(0, 0, len(0))), SE3(Vector3(0, 0, 0)));
		this->addMate(_joints[1], "L1", "L2", SE3(SO3::RotX(-PI_HALF)*SO3::RotZ(-PI_HALF), Vector3(len(1), 0, 0)), SE3(Vector3(0, 0, 0)));
		this->addMate(_joints[2], "L2", "L3", SE3(Vector3(len(2), 0, 0)), SE3(Vector3(0, 0, 0)));
		this->addMate(_joints[3], "L3", "L4", SE3(SO3::RotX(-PI_HALF), Vector3(len(3), len(4), 0)), SE3(Vector3(0, 0, 0)));
		this->addMate(_joints[4], "L4", "L5", SE3(SO3::RotX(PI_HALF), Vector3(0, 0, 0)), SE3(Vector3(0, 0, 0)));
		this->addMate(_joints[5], "L5", "L6", SE3(SO3::RotX(-PI_HALF), Vector3(0, 0, 0)), SE3(Vector3(0, 0, 0)));

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
		vector<Inertia, Eigen::aligned_allocator<Inertia>> G(6);
		Vector3 p;
		for (int i = 0; i < 6; i++)
		{
			I(0, 0) = inr_info(i, 0);
			I(1, 1) = inr_info(i, 1);
			I(2, 2) = inr_info(i, 2);
			I(0, 1) = I(1, 0) = inr_info(i, 3);
			I(0, 2) = I(2, 0) = inr_info(i, 4);
			I(1, 2) = I(2, 1) = inr_info(i, 5);
			p = -mx.col(i);
			G[i] = Inertia(I, p, m(i));
		}
		for (unsigned int i = 0; i < this->getMateList().size(); i++)
			this->getLinkPtr("L" + to_string(i + 1))->setInertia(G[i]);

		// set motor parameters
		VectorX L(6);
		VectorX R(6);
		VectorX kt(6);
		VectorX kb(6);
		VectorX J(6);
		VectorX gearRatio(6);
		L << 3.5, 3.5, 5.2, 8, 8, 8;
		L *= 1e-3;

		R << 0.58, 0.58, 0.8, 2.9, 2.9, 7.5;
		kt << 0.73, 0.73, 0.5, 0.4, 0.4, 0.39;
		kb = kt;
		J << 1.06, 1.06, 0.13, 0.044, 0.044, 0.027;
		J *= 1e-3;
		gearRatio << 147, 153, 153, 76.95, 80, 51;
		for (int i = 0; i < 6; i++)
		{
			static_pointer_cast<MotorJoint> (this->getJointPtrByMateIndex(i))->setInductance(L(i));
			static_pointer_cast<MotorJoint> (this->getJointPtrByMateIndex(i))->setResistance(R(i));
			static_pointer_cast<MotorJoint> (this->getJointPtrByMateIndex(i))->setMotorConstant(kt(i));
			static_pointer_cast<MotorJoint> (this->getJointPtrByMateIndex(i))->setBackEMFConstant(kb(i));
			static_pointer_cast<MotorJoint> (this->getJointPtrByMateIndex(i))->setRotorInertia(J(i));
			static_pointer_cast<MotorJoint> (this->getJointPtrByMateIndex(i))->setGearRatio(gearRatio(i));
		}



		// set constraints

		VectorX taumax(6);
		VectorX taumin(6);
		VectorX qmax(6);
		VectorX qmin(6);
		VectorX qdotmax(6);
		VectorX qdotmin(6);
		VectorX qddotmax(6);
		VectorX qddotmin(6);
		VectorX qdddotmax(6);
		VectorX qdddotmin(6);
		VectorX kv(6);
		VectorX kc(6);
		VectorX smax(6);
		VectorX imax(6);

		qmax << 175, 90, 70, 180, 135, 360;
		qmax *= PI / 180;

		qmin << 175, 100, 145, 180, 135, 360;
		qmin *= -PI / 180;
		qdotmax << 100, 80, 140, 290, 290, 440;
		qdotmax *= PI / 180;

		qddotmax = 5 * qdotmax;             // user defined
		qddotmin = -qddotmax;
		qdddotmax = 300 * VectorX::Ones(6);          // user defined
		qdddotmin = -qdddotmax;
		taumax = 3000 * VectorX::Ones(6);          // user defined

		kv << 105.4116, 107.5105, 8.1031, 6.5430, 11.3551, 4.2050;
		kc << 92.3012, 117.3703, 57.8389, 10.0524, 24.6819, 20.32;
		smax << 3000, 3000, 4500, 4500, 4500, 4500;
		smax *= 2 * PI / 60;    // smax = [5000, 5000, 5000, 5000, 5000, 50000] * 2 * pi / 60,    % motor maximum
		imax << 39, 39, 20, 12, 12, 7.2;

		VectorX temptau = (imax.cwiseProduct(gearRatio)).cwiseProduct(kt);
		VectorX tempvel = smax.cwiseQuotient(gearRatio);
		taumax = taumax.cwiseMin(temptau);
		qdotmax = qdotmax.cwiseMin(tempvel);
		qdotmin = -qdotmax;
		taumin = -taumax;
		for (unsigned int i = 0; i < 6; i++)
		{
			this->getJointPtrByMateIndex(i)->setConstDamper(kv.segment(i, 1));
			this->getJointPtrByMateIndex(i)->setConstFriction(kc.segment(i, 1));
			this->getJointPtrByMateIndex(i)->setLimitPos(qmin.segment(i, 1), qmax.segment(i, 1));
			this->getJointPtrByMateIndex(i)->setLimitVel(qdotmin.segment(i, 1), qdotmax.segment(i, 1));
			this->getJointPtrByMateIndex(i)->setLimitAcc(qddotmin.segment(i, 1), qddotmax.segment(i, 1));
			this->getJointPtrByMateIndex(i)->setLimitJerk(qdddotmin.segment(i, 1), qdddotmax.segment(i, 1));
			this->getJointPtrByMateIndex(i)->setLimitInput(taumin.segment(i, 1), taumax.segment(i, 1));
		}

		this->completeAssembling(_links[0]->getName());
	}

	void addMeshFiles()
	{
		StatePtr efortState = this->makeState();
		Kinematics::solveForwardKinematics(static_cast<socAssembly&>(*this), *efortState, State::LINKS_POS);
		Vector4	orange(254 / 255.0, 193 / 255.0, 27 / 255.0, 1.0), black(55 / 255.0, 55 / 255.0, 55 / 255.0, 1.0);
		//for (unsigned int i = 0; i < _links.size(); i++)
		//{
		//	shared_ptr<Box> boxShape(new Box(0.2, 0.2, 0.2));
		//	this->getLinkPtr(i)->addDrawingShapes(boxShape);
		//}
		for (unsigned int i = 0; i < 1; i++)
		{
			auto linkStatePtr = efortState->getLinkState("L0");
			shared_ptr<Mesh> STL_file(new Mesh(string("../Data/CAD/efort_robot/LINK0_0") + std::to_string(i + 1) + string(".STL")));
			STL_file->setFrame(linkStatePtr._T.inverse());
			STL_file->setDimension(1);
			STL_file->setColor(orange);
			this->getLinkPtr("L0")->addDrawingShapes(STL_file);
		}
		for (unsigned int i = 0; i < 6; i++)
		{
			auto linkStatePtr = efortState->getLinkState("L1");
			shared_ptr<Mesh> STL_file(new Mesh(string("../Data/CAD/efort_robot/LINK1_0") + std::to_string(i + 1) + string(".STL")));
			STL_file->setFrame(linkStatePtr._T.inverse());
			STL_file->setDimension(1);
			if(i==1||i==2||i==3||i==4)
				STL_file->setColor(black);
			else
				STL_file->setColor(orange);
			this->getLinkPtr("L1")->addDrawingShapes(STL_file);
		}
		for (unsigned int i = 0; i < 1; i++)
		{
			auto linkStatePtr = efortState->getLinkState("L2");
			shared_ptr<Mesh> STL_file(new Mesh(string("../Data/CAD/efort_robot/LINK2_0") + std::to_string(i + 1) + string(".STL")));
			STL_file->setFrame(linkStatePtr._T.inverse());
			STL_file->setDimension(1);
			STL_file->setColor(orange);
			this->getLinkPtr("L2")->addDrawingShapes(STL_file);
		}
		for (unsigned int i = 0; i < 7; i++)
		{
			auto linkStatePtr = efortState->getLinkState("L3");
			shared_ptr<Mesh> STL_file(new Mesh(string("../Data/CAD/efort_robot/LINK3_0") + std::to_string(i + 1) + string(".STL")));
			STL_file->setFrame(linkStatePtr._T.inverse());
			STL_file->setDimension(1);
			if (i == 1 || i == 2 || i == 3 || i == 4 ||i==5)
				STL_file->setColor(black);
			else
				STL_file->setColor(orange);
			this->getLinkPtr("L3")->addDrawingShapes(STL_file);
		}
		for (unsigned int i = 0; i < 8; i++)
		{
			auto linkStatePtr = efortState->getLinkState("L4");
			shared_ptr<Mesh> STL_file(new Mesh(string("../Data/CAD/efort_robot/LINK4_0") + std::to_string(i + 1) + string(".STL")));
			STL_file->setFrame(linkStatePtr._T.inverse());
			STL_file->setDimension(1);
			if (i == 0)
				STL_file->setColor(black);
			else
				STL_file->setColor(orange);
			this->getLinkPtr("L4")->addDrawingShapes(STL_file);
		}
		for (unsigned int i = 0; i < 3; i++)
		{
			auto linkStatePtr = efortState->getLinkState("L5");
			shared_ptr<Mesh> STL_file(new Mesh(string("../Data/CAD/efort_robot/LINK5_0") + std::to_string(i + 1) + string(".STL")));
			STL_file->setFrame(linkStatePtr._T.inverse());
			STL_file->setDimension(1);
			STL_file->setColor(orange);
			this->getLinkPtr("L5")->addDrawingShapes(STL_file);
		}
		for (unsigned int i = 0; i < 1; i++)
		{
			auto linkStatePtr = efortState->getLinkState("L6");
			shared_ptr<Mesh> STL_file(new Mesh(string("../Data/CAD/efort_robot/LINK6_0") + std::to_string(i + 1) + string(".STL")));
			STL_file->setFrame(linkStatePtr._T.inverse());
			STL_file->setDimension(1);
			if (i == 0)
				STL_file->setColor(black);
			else
				STL_file->setColor(orange);
			this->getLinkPtr("L6")->addDrawingShapes(STL_file);
		}
	}

	vector<shared_ptr< Link >>	_links;
	vector<shared_ptr< Joint >> _joints;
};