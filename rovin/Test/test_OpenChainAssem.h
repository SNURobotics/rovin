#pragma once

#include <vector>
#include <string>
#include <rovin/Model/Assembly.h>
#include <rovin/Model/RevoluteJoint.h>
#include <rovin/Dynamics/Kinematics.h>
#include <rovin/Dynamics/Dynamics.h>
#include <rovin/utils/Diagnostic.h>

using namespace std;
using namespace rovin::Math;
using namespace rovin::Model;

template <unsigned int DOF>
class serialChain :public socAssembly
{
public:
	serialChain()
		:_dof(DOF), socAssembly("Open Chain Assembly")
	{
		_links.resize(DOF + 1);
		_joints.resize(DOF);
		assembleModel();
	}

	void	assembleModel()
	{

		for (unsigned int i = 0; i < _links.size(); i++)
		{
			_links[i] = shared_ptr< Link >(new Link(string("L")+ std::to_string(i)));
			this->addLink(_links[i]);
		}

		for (unsigned int i = 0; i < _joints.size(); i++)
		{
			_joints[i] = shared_ptr< Joint >(new RevoluteJoint(string("J") + std::to_string(i)));
			this->addMate(_joints[i], _links[i]->getName(), _links[i + 1]->getName(), SE3(Vector3(1, 0, 0)), SE3(Vector3(1, 0, 0)));
		}
		this->completeAssembling(_links[0]->getName());
	}

	unsigned int _dof;
	vector<shared_ptr< Link >>	_links;
	vector<shared_ptr< Joint >> _joints;

};
