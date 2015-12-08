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

		this->completeAssembling(_links[0]->getName());
	}

	void addMeshFiles()
	{
		StatePtr efortState = this->makeState();
		Kinematics::solveForwardKinematics(static_cast<socAssembly&>(*this), *efortState, State::LINKS_POS);
		SE3 offset(Vector3(-240.95 / 1000, 346.10 / 1000, -110.30 / 1000));
		for (unsigned int i = 0; i < 4; i++)
		{
			auto linkStatePtr = efortState->getLinkState("L0");
			shared_ptr<Mesh> STL_file(new Mesh(string("../Data/CAD/efort_robot/LINK0_0") + std::to_string(i + 1) + string(".STL")));
			STL_file->setFrame(linkStatePtr._T.inverse()*offset* SE3(SO3::RotX(PI_HALF)));
			STL_file->setDimension(0.001);
			this->getLinkPtr("L0")->addDrawingShapes(STL_file);
		}
		for (unsigned int i = 0; i < 6; i++)
		{
			auto linkStatePtr = efortState->getLinkState("L1");
			shared_ptr<Mesh> STL_file(new Mesh(string("../Data/CAD/efort_robot/LINK1_0") + std::to_string(i + 1) + string(".STL")));
			STL_file->setFrame(linkStatePtr._T.inverse()*offset* SE3(SO3::RotX(PI_HALF)));
			STL_file->setDimension(0.001);
			this->getLinkPtr("L1")->addDrawingShapes(STL_file);
		}
		for (unsigned int i = 0; i < 1; i++)
		{
			auto linkStatePtr = efortState->getLinkState("L2");
			shared_ptr<Mesh> STL_file(new Mesh(string("../Data/CAD/efort_robot/LINK2_0") + std::to_string(i + 1) + string(".STL")));
			STL_file->setFrame(linkStatePtr._T.inverse()*offset* SE3(SO3::RotX(PI_HALF)));
			STL_file->setDimension(0.001);
			this->getLinkPtr("L2")->addDrawingShapes(STL_file);
		}
		for (unsigned int i = 0; i < 7; i++)
		{
			auto linkStatePtr = efortState->getLinkState("L3");
			shared_ptr<Mesh> STL_file(new Mesh(string("../Data/CAD/efort_robot/LINK3_0") + std::to_string(i + 1) + string(".STL")));
			STL_file->setFrame(linkStatePtr._T.inverse()*offset* SE3(SO3::RotX(PI_HALF)));
			STL_file->setDimension(0.001);
			this->getLinkPtr("L3")->addDrawingShapes(STL_file);
		}
		for (unsigned int i = 0; i < 8; i++)
		{
			auto linkStatePtr = efortState->getLinkState("L4");
			shared_ptr<Mesh> STL_file(new Mesh(string("../Data/CAD/efort_robot/LINK4_0") + std::to_string(i + 1) + string(".STL")));
			STL_file->setFrame(linkStatePtr._T.inverse()*offset* SE3(SO3::RotX(PI_HALF)));
			STL_file->setDimension(0.001);
			this->getLinkPtr("L4")->addDrawingShapes(STL_file);
		}
		for (unsigned int i = 0; i < 3; i++)
		{
			auto linkStatePtr = efortState->getLinkState("L5");
			shared_ptr<Mesh> STL_file(new Mesh(string("../Data/CAD/efort_robot/LINK5_0") + std::to_string(i + 1) + string(".STL")));
			STL_file->setFrame(linkStatePtr._T.inverse()*offset* SE3(SO3::RotX(PI_HALF)));
			STL_file->setDimension(0.001);
			this->getLinkPtr("L5")->addDrawingShapes(STL_file);
		}
		for (unsigned int i = 0; i < 1; i++)
		{
			auto linkStatePtr = efortState->getLinkState("L6");
			shared_ptr<Mesh> STL_file(new Mesh(string("../Data/CAD/efort_robot/LINK6_0") + std::to_string(i + 1) + string(".STL")));
			STL_file->setFrame(linkStatePtr._T.inverse()*offset* SE3(SO3::RotX(PI_HALF)));
			STL_file->setDimension(0.001);
			this->getLinkPtr("L6")->addDrawingShapes(STL_file);
		}
	}

	vector<shared_ptr< Link >>	_links;
	vector<shared_ptr< Joint >> _joints;
};