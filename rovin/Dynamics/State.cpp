#include "State.h"

using namespace std;

namespace rovin
{
	namespace Dynamics
	{
		State::State(const Model::Assembly& model, const list< string >& activeJointList) : _activejoint_dof(0)
		{
			list< string > linkNameList = model.getLinkNameList();
			for (list< string >::iterator link_iter = linkNameList.begin(); link_iter != linkNameList.end(); link_iter++)
			{
				_linkState.insert(pair< string, LinkState >(*link_iter, LinkState()));
			}
			list< string > jointNmaeList = model.getLinkNameList();
			for (list< string >::iterator joint_iter = jointNmaeList.begin(); joint_iter != jointNmaeList.end(); joint_iter++)
			{
				unsigned int tmpdof = model.findJoint(*joint_iter)->getDOF();
				_jointState.insert(pair< string, JointState >(*joint_iter, JointState(tmpdof)));
				_total_dof += tmpdof;
			}

			for (list <string>::const_iterator activeJoint_iter = activeJointList.begin(); activeJoint_iter != activeJointList.end(); activeJoint_iter++)
			{
				if (!addActiveJoint(*activeJoint_iter))
				{
					utils::Log("State�� �����ϴ� ���߿� ������ ������ϴ�.", true);
				}
			}
		}

		bool State::addActiveJoint(const string& activeJoint)
		{
			for (list< string >::iterator iter = _activeJointList.begin(); iter != _activeJointList.end(); iter++)
			{
				if (*iter == activeJoint)
				{
					utils::Log("�߰��ϰ� ���� joint�� �̹� active joint list�� ��� �ֽ��ϴ�.", false);
					return false;
				}
			}
			if (utils::Log(_jointState.find(activeJoint) == _jointState.end(), "�߰� �ϰ� ���� �̸��� ���� joint�� ���� ���� �ʽ��ϴ�.", false))
			{
				return false;
			}
			_activeJointList.push_back(activeJoint);
			_activejoint_dof += getJointState(activeJoint).dof;
			return true;
		}

		bool State::eraseActiveJoint(const string& activeJoint)
		{
			for (list< string >::iterator iter = _activeJointList.begin(); iter != _activeJointList.end(); iter++)
			{
				if (*iter == activeJoint)
				{
					_activeJointList.erase(iter);
					_activejoint_dof -= getJointState(activeJoint).dof;
					return true;
				}
			}
			utils::Log("�߰��ϰ� ���� joint�� active joint list�� ��� ���� �ʽ��ϴ�.", false);
			return false;
		}
		
		std::list< Math::VectorX > State::vector2list(const Math::VectorX& _q)
		{
			utils::Log(_q.size() != _activejoint_dof, "State�� �������� q�� �� �������� �ٸ��ϴ�.", true);

			unsigned int index = 0, dof;
			std::list< Math::VectorX > tmp = std::list< Math::VectorX >();
			for (list< string >::iterator iter = _activeJointList.begin(); iter != _activeJointList.end(); iter++)
			{
				dof = getJointState(*iter).dof;
				tmp.push_back(_q.block(index, 0, dof, 1));
				index += dof;
			}
			return tmp;
		}
		
		Math::VectorX State::list2vector(const std::list< Math::VectorX >& _q)
		{
			utils::Log(_q.size() != _jointState.size(), "State�� ������ q�� ������ �ٸ��ϴ�.", true);

			unsigned int dof = 0;
			Math::VectorX tmp(_activejoint_dof);

			for (list< Math::VectorX >::const_iterator iter = _q.begin(); iter != _q.end(); iter++)
			{
				for (int i = 0; i < (*iter).size(); i++, dof++)
				{
					tmp(dof) = (*iter)(i);
				}
			}

			utils::Log(dof != _activejoint_dof, "State�� �������� q�� �� �������� �ٸ��ϴ�.", true);
			return tmp;
		}
		
		void State::setJoint_q(const Math::VectorX& _q)
		{
			utils::Log(_q.size() != _activejoint_dof, "State�� ������ q�� ������ �ٸ��ϴ�.", true);

			list< Math::VectorX > _qlist = vector2list(_q);
			list< string >::iterator iter;
			list< Math::VectorX >::iterator _qiter;
			for (iter = _activeJointList.begin(), _qiter = _qlist.begin(); iter != _activeJointList.end(); iter++, _qiter++)
			{
				getJointState(*iter).q = *_qiter;
			}
		}

		void State::setJoint_qdot(const Math::VectorX& _qdot)
		{
			utils::Log(_qdot.size() != _activejoint_dof, "State�� ������ qdot�� ������ �ٸ��ϴ�.", true);

			list< Math::VectorX > _qdotlist = vector2list(_qdot);
			list< string >::iterator iter;
			list< Math::VectorX >::iterator _qdotiter;
			for (iter = _activeJointList.begin(), _qdotiter = _qdotlist.begin(); iter != _activeJointList.end(); iter++, _qdotiter++)
			{
				getJointState(*iter).qdot = *_qdotiter;
			}
		}

		void State::setJoint_tau(const Math::VectorX& _tau)
		{
			utils::Log(_tau.size() != _activejoint_dof, "State�� ������ tau�� ������ �ٸ��ϴ�.", true);

			list< Math::VectorX > _taulist = vector2list(_tau);
			list< string >::iterator iter;
			list< Math::VectorX >::iterator _tauiter;
			for (iter = _activeJointList.begin(), _tauiter = _taulist.begin(); iter != _activeJointList.end(); iter++, _tauiter++)
			{
				getJointState(*iter).tau = *_tauiter;
			}
		}
	}
}