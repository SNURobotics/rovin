#include "State.h"

using namespace std;

namespace rovin
{
	namespace Dynamics
	{
		State::State(const System& system, const list< string >& activeJointList) : _activejoint_dof(0)
		{
			const vector< string > linkNameList = system.getLinkList();
			_linkState.clear();
			int i = 0;
			for (vector< string >::const_iterator link_iter = linkNameList.begin(); link_iter != linkNameList.end(); link_iter++, i++)
			{
				_linkState.push_back(LinkState());
				_linkMap.insert(pair< string, unsigned int >(*link_iter, i));
			}
			const vector< string > jointNmaeList = system.getJointList();
			_jointState.clear();
			_jointSystemIndex.clear();
			_activeJoint.clear();
			i = 0;
			for (vector< string >::const_iterator joint_iter = jointNmaeList.begin(); joint_iter != jointNmaeList.end(); joint_iter++, i++)
			{
				unsigned int tmpdof = system.getAssembly().findJoint(*joint_iter)->getDOF();
				_jointState.push_back(JointState(tmpdof));
				_jointState[i].index = _total_dof;
				_jointMap.insert(pair< string, unsigned int >(*joint_iter, i));
				_jointSystemIndex.push_back(_total_dof);
				
				_activeJoint.push_back(false);
				_passiveJointList.push_back(pair< string, unsigned int >(*joint_iter, i));

				_total_dof += tmpdof;
			}
	
			for (list <string>::const_iterator activeJoint_iter = activeJointList.begin(); activeJoint_iter != activeJointList.end(); activeJoint_iter++)
			{
				if (!addActiveJoint(*activeJoint_iter))
				{
					utils::Log("State를 구성하는 도중에 에러가 생겼습니다.", true);
				}
			}
		}

		bool State::addActiveJoint(const string& joint_name)
		{
			if (utils::Log(_jointMap.find(joint_name) == _jointMap.end(), "추가 하고 싶은 이름을 갖는 joint는 존재 하지 않습니다.", false))
			{
				return false;
			}
			
			for (list< pair< string, unsigned int >>::iterator iter = _passiveJointList.begin(); iter != _passiveJointList.end(); iter++)
			{
				if (iter->first == joint_name)
				{
					_activeJoint[iter->second] = true;

					_activeJointList.push_back(pair< string, unsigned int >(joint_name, iter->second));
					_jointState[iter->second].index = _activejoint_dof;
					_activejoint_dof += _jointState[iter->second].dof;

					iter = _passiveJointList.erase(iter);
					unsigned int deDof = _jointState[iter->second].dof;
					for (; iter != _passiveJointList.end(); iter++)
					{
						_jointState[iter->second].index -= deDof;
					}
					return true;
				}
			}
			utils::Log("추가하고 싶은 joint는 이미 active joint list에 들어 있습니다.", false);
			return false;
		}

		bool State::eraseActiveJoint(const string& joint_name)
		{
			if (utils::Log(_jointMap.find(joint_name) == _jointMap.end(), "추가 하고 싶은 이름을 갖는 joint는 존재 하지 않습니다.", false))
			{
				return false;
			}
			for (list< pair< string, unsigned int >>::iterator iter = _activeJointList.begin(); iter != _activeJointList.end(); iter++)
			{
				if (iter->first == joint_name)
				{
					_activeJoint[iter->second] = false;

					_passiveJointList.push_back(pair< string, unsigned int >(joint_name, iter->second));
					_jointState[iter->second].index = _total_dof - _activejoint_dof;
					_activejoint_dof -= _jointState[iter->second].dof;

					iter = _activeJointList.erase(iter);
					for (; iter != _activeJointList.end(); iter++)
					{
						_jointState[iter->second].index -= _jointState[iter->second].dof;
					}
					return true;
				}
			}
			utils::Log("추가하고 싶은 joint는 active joint list에 들어 있지 않습니다.", false);
			return false;
		}

		unsigned int State::getDof(const System::RETURN_STATE& return_state)
		{
			if (return_state == System::SYSTEMJOINT)	return getTotalDof();
			if (return_state == System::WHOLEJOINT)		return getTotalDof();
			if (return_state == System::ACTIVEJOINT)	return getActiveJointDof();
			if (return_state == System::PASSIVEJOINT)	return getTotalDof() - getActiveJointDof();
			return -1;
		}

		void State::writeColumns(Math::MatrixX& target, const Math::MatrixX& value, const unsigned int& row, const unsigned int& joint_num,
			const System::RETURN_STATE& return_state)
		{
			unsigned int index = getTotalDof();
			if (return_state == System::SYSTEMJOINT)
			{
				index = _jointSystemIndex[joint_num];
			}
			else if (return_state == System::WHOLEJOINT)
			{
				if (isActiveJoint(joint_num))
				{
					index = getJointState(joint_num).index;
				}
				else
				{
					index = getJointState(joint_num).index + getActiveJointDof();
				}
			}
			else if (return_state == System::ACTIVEJOINT)
			{
				if (isActiveJoint(joint_num))
				{
					index = getJointState(joint_num).index;
				}
			}
			else if (return_state == System::PASSIVEJOINT)
			{
				if (!isActiveJoint(joint_num))
				{
					index = getJointState(joint_num).index;
				}
			}

			if (index < getTotalDof())
			{
				int valuecol = value.cols();
				int valuerow = value.rows();

				target.block(row, index, valuerow, valuecol) = value;
			}
		}

		void State::setActiveJoint_q(const Math::VectorX& q)
		{
			for (list< pair< string, unsigned int >>::iterator iter = _activeJointList.begin(); iter != _activeJointList.end(); iter++)
			{
				_jointState[iter->second].q = q.block(_jointState[iter->second].index, 0, _jointState[iter->second].dof, 1);
			}
		}

		void State::setPassiveJoint_q(const Math::VectorX& q)
		{
			for (list< pair< string, unsigned int >>::iterator iter = _passiveJointList.begin(); iter != _passiveJointList.end(); iter++)
			{
				_jointState[iter->second].q = q.block(_jointState[iter->second].index, 0, _jointState[iter->second].dof, 1);
			}
		}

		void State::addPassiveJoint_q(const Math::VectorX& q)
		{
			for (list< pair< string, unsigned int >>::iterator iter = _passiveJointList.begin(); iter != _passiveJointList.end(); iter++)
			{
				_jointState[iter->second].q += q.block(_jointState[iter->second].index, 0, _jointState[iter->second].dof, 1);
			}
		}
	}
}