#include "State.h"

#include <rovin/utils/Diagnostic.h>

using namespace std;

namespace rovin
{
	namespace Model
	{
		State::LinkState::LinkState()
		{
			_V.setZero();
		}

		State::JointState::JointState(const unsigned int& dof)
		{
			_dof = dof;

			_q = Math::VectorX(dof);
			_qdot = Math::VectorX(dof);
			_qddot = Math::VectorX(dof);

			_tau = Math::VectorX(dof);

			_q.setZero();
			_qdot.setZero();
			_qddot.setZero();
			_tau.setZero();
			_constraintF.setZero();

			_T = vector< Math::SE3 >(dof);
		}

		State::State(const vector< string >& linkNameList, const vector< pair< string, unsigned int >>& jointNameList)
		{
			_totalJointDof = _activeJointDof = 0;

			for (unsigned int i = 0; i < linkNameList.size(); i++)
			{
				_linkState.push_back(LinkState());
				_linkName.push_back(linkNameList[i]);
				_linkIndexMap.insert(pair< string, unsigned int>(linkNameList[i], i));
			}
			for (unsigned int i = 0; i < jointNameList.size(); i++)
			{
				_jointState.push_back(JointState(jointNameList[i].second));
				_jointName.push_back(jointNameList[i].first);
				_jointIndexMap.insert(pair< string, unsigned int>(jointNameList[i].first, i));

				_isActiveJoint.push_back(false);
				_passiveJointList.push_back(i);
				_assemIndex.push_back(_totalJointDof);
				_stateIndex.push_back(_totalJointDof);
				_totalJointDof += jointNameList[i].second;
			}
		}

		unsigned int State::getTotalJointDof() const
		{
			return _totalJointDof;
		}

		unsigned int State::getActiveJointDof() const
		{
			return _activeJointDof;
		}

		State::LinkState& State::getLinkState(const unsigned int linkIndex)
		{
			return _linkState[linkIndex];
		}

		State::LinkState& State::getLinkState(const string& linkName)
		{
			return _linkState[getLinkIndex(linkName)];
		}

		State::JointState& State::getJointState(const unsigned int jointIndex)
		{
			return _jointState[jointIndex];
		}

		State::JointState& State::getJointStateByMateIndex(const unsigned int mateIndex)
		{
			return _jointState[mateIndex];
		}

		State::JointState& State::getJointState(const string& jointName)
		{
			return _jointState[getJointIndex(jointName)];
		}

		const State::LinkState& State::getLinkState(const unsigned int linkIndex) const
		{
			return _linkState[linkIndex];
		}

		const State::LinkState& State::getLinkState(const string& linkName) const
		{
			return _linkState[getLinkIndex(linkName)];
		}

		const State::JointState& State::getJointState(const unsigned int jointIndex) const
		{
			return _jointState[jointIndex];
		}

		const State::JointState& State::getJointStateByMateIndex(const unsigned int mateIndex) const
		{
			return _jointState[mateIndex];
		}

		const State::JointState& State::getJointState(const string& jointName) const
		{
			return _jointState[getJointIndex(jointName)];
		}

		unsigned int State::getLinkIndex(const string& linkName) const
		{
			return _linkIndexMap.find(linkName)->second;
		}

		unsigned int State::getJointIndex(const string& jointName) const
		{
			return _jointIndexMap.find(jointName)->second;
		}

		unsigned int State::getJointIndexByMateIndex(const unsigned int& mateIdx) const
		{
			return mateIdx;
		}

		void State::addActiveJoint(const std::string& jointName)
		{
			utils::Log(_isActiveJoint[getJointIndex(jointName)], "�̹� active joint�� �����Ǿ��ֽ��ϴ�.", true);

			bool flag = false;
			unsigned int Idx;
			for (unsigned int i = 0; i < _passiveJointList.size(); i++)
			{
				if (flag == true)
				{
					_stateIndex[_passiveJointList[i]] -= _jointState[_passiveJointList[Idx]]._dof;
				}
				if (_jointName[_passiveJointList[i]].compare(jointName) == 0)
				{
					Idx = i;
					flag = true;
				}
			}

			_isActiveJoint[_passiveJointList[Idx]] = true;
			_activeJointList.push_back(_passiveJointList[Idx]);
			_stateIndex[_passiveJointList[Idx]] = _activeJointDof;
			_activeJointDof += _jointState[_passiveJointList[Idx]]._dof;

			_passiveJointList.erase(_passiveJointList.begin() + Idx);
		}

		void State::addActiveJoint(const std::vector< std::string >& jointNmaeList)
		{
			for (unsigned int i = 0; i < jointNmaeList.size(); i++)
			{
				addActiveJoint(jointNmaeList[i]);
			}
		}

		void State::eraseActiveJoint(const std::string& jointName)
		{
			utils::Log(_isActiveJoint[getJointIndex(jointName)], "Active joint�� �ƴմϴ�.", true);

			bool flag = false;
			unsigned int Idx;
			for (unsigned int i = 0; i < _activeJointList.size(); i++)
			{
				if (flag == true)
				{
					_stateIndex[_activeJointList[i]] -= _jointState[_activeJointList[Idx]]._dof;
				}
				if (_jointName[_activeJointList[i]].compare(jointName) == 0)
				{
					Idx = i;
					flag = true;
				}
			}
			
			_passiveJointList.push_back(_activeJointList[Idx]);
			_stateIndex[_activeJointList[Idx]] = _totalJointDof - _activeJointDof;
			_activeJointDof -= _jointState[_activeJointList[Idx]]._dof;

			_isActiveJoint[_activeJointList[Idx]] = false;
			_activeJointList.erase(_activeJointList.begin() + Idx);
		}

		vector< string > State::getJointList() const
		{
			vector< string > jointList;
			for (unsigned int i = 0; i < _activeJointList.size(); i++)
			{
				jointList.push_back(_jointName[_activeJointList[i]]);
			}
			for (unsigned int i = 0; i < _passiveJointList.size(); i++)
			{
				jointList.push_back(_jointName[_passiveJointList[i]]);
			}
			return jointList;
		}

		vector< string > State::getActiveJointList() const
		{
			vector< string > jointList;
			for (unsigned int i = 0; i < _activeJointList.size(); i++)
			{
				jointList.push_back(_jointName[_activeJointList[i]]);
			}
			return jointList;
		}

		vector< string > State::getPassiveJointList() const
		{
			vector< string > jointList;
			for (unsigned int i = 0; i < _passiveJointList.size(); i++)
			{
				jointList.push_back(_jointName[_passiveJointList[i]]);
			}
			return jointList;
		}

		void State::setActiveJointq(const Math::VectorX& q)
		{
			for (unsigned int i = 0; i < _activeJointList.size(); i++)
			{
				_jointState[_activeJointList[i]]._q = q.block(_stateIndex[_activeJointList[i]], 0, _jointState[_activeJointList[i]]._dof, 1);
			}
		}

		void State::setPassiveJointq(const Math::VectorX& q)
		{
			for (unsigned int i = 0; i < _passiveJointList.size(); i++)
			{
				_jointState[_passiveJointList[i]]._q = q.block(_stateIndex[_passiveJointList[i]], 0, _jointState[_passiveJointList[i]]._dof, 1);
			}
		}

		void State::addActiveJointq(const Math::VectorX& q)
		{
			for (unsigned int i = 0; i < _activeJointList.size(); i++)
			{
				_jointState[_activeJointList[i]]._q += q.block(_stateIndex[_activeJointList[i]], 0, _jointState[_activeJointList[i]]._dof, 1);
			}
		}

		void State::addPassiveJointq(const Math::VectorX& q)
		{
			for (unsigned int i = 0; i < _passiveJointList.size(); i++)
			{
				_jointState[_passiveJointList[i]]._q += q.block(_stateIndex[_passiveJointList[i]], 0, _jointState[_passiveJointList[i]]._dof, 1);
			}
		}

		unsigned int State::returnDof(const RETURN_STATE& return_state) const
		{
			if (return_state == RETURN_STATE::STATEJOINT)
			{
				return _totalJointDof;
			}
			else if (return_state == RETURN_STATE::ACTIVEJOINT)
			{
				return _activeJointDof;
			}
			else if (return_state == RETURN_STATE::PASSIVEJOINT)
			{
				return _totalJointDof - _activeJointDof;
			}
			return _totalJointDof;
		}

		void State::writeReturnMatrix(Math::MatrixX& returnMatrix, const Math::MatrixX& value, const unsigned int startRow, const unsigned int jointIndex, const RETURN_STATE& return_state) const
		{
			unsigned int Idx = _assemIndex[jointIndex];
			if (return_state == RETURN_STATE::STATEJOINT)
			{
				if (_isActiveJoint[jointIndex])
				{
					Idx = _stateIndex[jointIndex];
				}
				else
				{
					Idx = _stateIndex[jointIndex] + _activeJointDof;
				}
			}
			else if (return_state == RETURN_STATE::ACTIVEJOINT || return_state == RETURN_STATE::PASSIVEJOINT)
			{
				Idx = _stateIndex[jointIndex];
			}
			returnMatrix.block(startRow, Idx, value.rows(), value.cols()) = value;
		}

		void State::writeReturnVector(Math::VectorX& returnVector, const Math::VectorX& value, const unsigned int jointIndex, const RETURN_STATE& return_state) const
		{
			unsigned int Idx = _assemIndex[jointIndex];
			if (return_state == RETURN_STATE::STATEJOINT)
			{
				if (_isActiveJoint[jointIndex])
				{
					Idx = _stateIndex[jointIndex];
				}
				else
				{
					Idx = _stateIndex[jointIndex] + _activeJointDof;
				}
			}
			else if (return_state == RETURN_STATE::ACTIVEJOINT || return_state == RETURN_STATE::PASSIVEJOINT)
			{
				Idx = _stateIndex[jointIndex];
			}
			returnVector.block(Idx, 0, value.size(), 1) = value;
		}
	}
}