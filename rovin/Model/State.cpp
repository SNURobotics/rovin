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

			if (dof != 0)
			{
				_q = Math::VectorX(dof);
				_qdot = Math::VectorX(dof);
				_qddot = Math::VectorX(dof);

				_tau = Math::VectorX(dof);

				_q.setZero();
				_qdot.setZero();
				_qddot.setZero();
				_tau.setZero();

				_T = vector< Math::SE3 >(dof);
				_J = Math::Matrix6X(6, dof);
				_JDot = Math::Matrix6X(6, dof);

				_JointReferenceFrame = -1;
				_TUpdated = _JUpdated = _JDotUpdated = false;
			}
			else
			{
				_TUpdated = _JUpdated = _JDotUpdated = true;
			}

			_constraintF.setZero();

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

			_JointReferenceFrame = -1;
			_TUpdated = _VUpdated = _VDotUpdated = false;
			_accumulatedJ = _accumulatedJDot = false;
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
			utils::Log(_isActiveJoint[getJointIndex(jointName)], "이미 active joint로 설정되어있습니다.", true);

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

					utils::Log(_jointState[_passiveJointList[Idx]]._dof == 0, "Active joint의 Dof는 0이면 안됩니다.", true);
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
			utils::Log(_isActiveJoint[getJointIndex(jointName)], "Active joint가 아닙니다.", true);

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
				_jointState[_activeJointList[i]].setq(q.block(_stateIndex[_activeJointList[i]], 0, _jointState[_activeJointList[i]]._dof, 1));
			}
			_accumulatedT = _accumulatedJ = _accumulatedJDot = false;
			needUpdate(true, true, true);
		}

		void State::setPassiveJointq(const Math::VectorX& q)
		{
			for (unsigned int i = 0; i < _passiveJointList.size(); i++)
			{
				if (_jointState[_passiveJointList[i]]._dof == 0) continue;
				_jointState[_passiveJointList[i]].setq(q.block(_stateIndex[_passiveJointList[i]], 0, _jointState[_passiveJointList[i]]._dof, 1));
			}
			_accumulatedT = _accumulatedJ = _accumulatedJDot = false;
			needUpdate(true, true, true);
		}

		void State::setActiveJointqdot(const Math::VectorX& qdot)
		{
			for (unsigned int i = 0; i < _activeJointList.size(); i++)
			{
				_jointState[_activeJointList[i]].setqdot(qdot.block(_stateIndex[_activeJointList[i]], 0, _jointState[_activeJointList[i]]._dof, 1));
			}
			_accumulatedJDot = false;
			needUpdate(false, true, true);
		}

		void State::setPassiveJointqdot(const Math::VectorX& qdot)
		{
			for (unsigned int i = 0; i < _passiveJointList.size(); i++)
			{
				if (_jointState[_passiveJointList[i]]._dof == 0) continue;
				_jointState[_passiveJointList[i]].setqdot(qdot.block(_stateIndex[_passiveJointList[i]], 0, _jointState[_passiveJointList[i]]._dof, 1));
			}
			_accumulatedJDot = false;
			needUpdate(false, true, true);
		}

		void State::setActiveJointqddot(const Math::VectorX& qddot)
		{
			for (unsigned int i = 0; i < _activeJointList.size(); i++)
			{
				_jointState[_activeJointList[i]].setqddot(qddot.block(_stateIndex[_activeJointList[i]], 0, _jointState[_activeJointList[i]]._dof, 1));
			}
			needUpdate(false, false, true);
		}

		void State::setPassiveJointqddot(const Math::VectorX& qddot)
		{
			for (unsigned int i = 0; i < _passiveJointList.size(); i++)
			{
				if (_jointState[_passiveJointList[i]]._dof == 0) continue;
				_jointState[_passiveJointList[i]].setqddot(qddot.block(_stateIndex[_passiveJointList[i]], 0, _jointState[_passiveJointList[i]]._dof, 1));
			}
			needUpdate(false, false, true);
		}

		void State::setActiveJointTorque(const Math::VectorX & torque)
		{
			for (unsigned int i = 0; i < _activeJointList.size(); i++)
				_jointState[_activeJointList[i]]._tau
				= torque.block(_stateIndex[_activeJointList[i]], 0, _jointState[_activeJointList[i]]._dof, 1);
		}

		void State::addActiveJointq(const Math::VectorX& q)
		{
			for (unsigned int i = 0; i < _activeJointList.size(); i++)
			{
				_jointState[_activeJointList[i]].addq(q.block(_stateIndex[_activeJointList[i]], 0, _jointState[_activeJointList[i]]._dof, 1));
			}
			_accumulatedT = _accumulatedJ = _accumulatedJDot = false;
			needUpdate(true, true, true);
		}

		void State::addPassiveJointq(const Math::VectorX& q)
		{
			for (unsigned int i = 0; i < _passiveJointList.size(); i++)
			{
				if (_jointState[_passiveJointList[i]]._dof == 0) continue;
				_jointState[_passiveJointList[i]].addq(q.block(_stateIndex[_passiveJointList[i]], 0, _jointState[_passiveJointList[i]]._dof, 1));
			}
			_accumulatedT = _accumulatedJ = _accumulatedJDot = false;
			needUpdate(true, true, true);
		}

		void State::addActiveJointqdot(const Math::VectorX& qdot)
		{
			for (unsigned int i = 0; i < _activeJointList.size(); i++)
			{
				_jointState[_activeJointList[i]].addqdot(qdot.block(_stateIndex[_activeJointList[i]], 0, _jointState[_activeJointList[i]]._dof, 1));
			}
			_accumulatedJDot = false;
			needUpdate(false, true, true);
		}

		void State::addPassiveJointqdot(const Math::VectorX& qdot)
		{
			for (unsigned int i = 0; i < _passiveJointList.size(); i++)
			{
				if (_jointState[_passiveJointList[i]]._dof == 0) continue;
				_jointState[_passiveJointList[i]].addqdot(qdot.block(_stateIndex[_passiveJointList[i]], 0, _jointState[_passiveJointList[i]]._dof, 1));
			}
			_accumulatedJDot = false;
			needUpdate(false, true, true);
		}

		void State::addActiveJointqddot(const Math::VectorX& qddot)
		{
			for (unsigned int i = 0; i < _activeJointList.size(); i++)
			{
				_jointState[_activeJointList[i]].addqddot(qddot.block(_stateIndex[_activeJointList[i]], 0, _jointState[_activeJointList[i]]._dof, 1));
			}
			needUpdate(false, false, true);
		}

		void State::addPassiveJointqddot(const Math::VectorX& qddot)
		{
			for (unsigned int i = 0; i < _passiveJointList.size(); i++)
			{
				if (_jointState[_passiveJointList[i]]._dof == 0) continue;
				_jointState[_passiveJointList[i]].addqddot(qddot.block(_stateIndex[_passiveJointList[i]], 0, _jointState[_passiveJointList[i]]._dof, 1));
			}
			needUpdate(false, false, true);
		}

		Math::VectorX State::getJointTorque(const TARGET_JOINT& target) const
		{
			Math::VectorX tau(getDOF(target));

			if (target == TARGET_JOINT::ASSEMJOINT)
				for (unsigned int i = 0; i < _jointState.size(); i++)
					for (unsigned j = 0; j < _jointState[i]._dof; j++)
						tau[_assemIndex[i] + j] = _jointState[i]._tau[j];


			if (target == TARGET_JOINT::STATEJOINT || target == TARGET_JOINT::ACTIVEJOINT)
				for (unsigned int i = 0; i < _activeJointList.size(); i++)
					for (unsigned int j = 0; j < _jointState[_activeJointList[i]]._dof; j++)
						tau[_stateIndex[_activeJointList[i]] + j] = _jointState[_activeJointList[i]]._tau[j];
			
			if (target == TARGET_JOINT::STATEJOINT || target == TARGET_JOINT::PASSIVEJOINT)
				for (unsigned int i = 0; i < _passiveJointList.size(); i++)
					for (unsigned int j = 0; j < _jointState[_passiveJointList[i]]._dof; j++)
						tau[_stateIndex[_passiveJointList[i]] + j] = _jointState[_passiveJointList[i]]._tau[j];

			return tau;
		}

		Math::VectorX State::getJointq(const TARGET_JOINT & target) const
		{
			Math::VectorX q(getDOF(target));
			if (target == TARGET_JOINT::ASSEMJOINT)
				for (unsigned int i = 0; i < _jointState.size(); i++)
					for (unsigned j = 0; j < _jointState[i]._dof; j++)
						q[_assemIndex[i] + j] = _jointState[i].getq(j);

			if (target == TARGET_JOINT::STATEJOINT || target == TARGET_JOINT::ACTIVEJOINT)
				for (unsigned int i = 0; i < _activeJointList.size(); i++)
					for (unsigned int j = 0; j < _jointState[_activeJointList[i]]._dof; j++)
						q[_stateIndex[_activeJointList[i]] + j] = _jointState[_activeJointList[i]].getq(j);
			
			if (target == TARGET_JOINT::STATEJOINT || target == TARGET_JOINT::PASSIVEJOINT)
				for (unsigned int i = 0; i < _passiveJointList.size(); i++)
					for (unsigned int j = 0; j < _jointState[_passiveJointList[i]]._dof; j++)
						q[_stateIndex[_passiveJointList[i]] + j] = _jointState[_passiveJointList[i]].getq(j);
			
			return q;
		}

		Math::VectorX State::getJointqdot(const TARGET_JOINT & target) const
		{
			Math::VectorX dq(getDOF(target));
			if (target == TARGET_JOINT::ASSEMJOINT)
				for (unsigned int i = 0; i < _jointState.size(); i++)
					for (unsigned j = 0; j < _jointState[i]._dof; j++)
						dq[_assemIndex[i] + j] = _jointState[i].getqdot(j);

			if (target == TARGET_JOINT::STATEJOINT || target == TARGET_JOINT::ACTIVEJOINT)
				for (unsigned int i = 0; i < _activeJointList.size(); i++)
					for (unsigned int j = 0; j < _jointState[_activeJointList[i]]._dof; j++)
						dq[_stateIndex[_activeJointList[i]] + j] = _jointState[_activeJointList[i]].getqdot(j);

			if (target == TARGET_JOINT::STATEJOINT || target == TARGET_JOINT::PASSIVEJOINT)
				for (unsigned int i = 0; i < _passiveJointList.size(); i++)
					for (unsigned int j = 0; j < _jointState[_passiveJointList[i]]._dof; j++)
						dq[_stateIndex[_passiveJointList[i]] + j] = _jointState[_passiveJointList[i]].getqdot(j);

			return dq;
		}

		Math::VectorX State::getJointqddot(const TARGET_JOINT & target) const
		{
			Math::VectorX ddq(getDOF(target));
			if (target == TARGET_JOINT::ASSEMJOINT)
				for (unsigned int i = 0; i < _jointState.size(); i++)
					for (unsigned j = 0; j < _jointState[i]._dof; j++)
						ddq[_assemIndex[i] + j] = _jointState[i].getqddot(j);

			if (target == TARGET_JOINT::STATEJOINT || target == TARGET_JOINT::ACTIVEJOINT)
				for (unsigned int i = 0; i < _activeJointList.size(); i++)
					for (unsigned int j = 0; j < _jointState[_activeJointList[i]]._dof; j++)
						ddq[_stateIndex[_activeJointList[i]] + j] = _jointState[_activeJointList[i]].getqddot(j);

			if (target == TARGET_JOINT::STATEJOINT || target == TARGET_JOINT::PASSIVEJOINT)
				for (unsigned int i = 0; i < _passiveJointList.size(); i++)
					for (unsigned int j = 0; j < _jointState[_passiveJointList[i]]._dof; j++)
						ddq[_stateIndex[_passiveJointList[i]] + j] = _jointState[_passiveJointList[i]].getqddot(j);

			return ddq;
		}

		unsigned int State::getDOF(const TARGET_JOINT& return_state) const
		{
			if (return_state == TARGET_JOINT::STATEJOINT)
			{
				return _totalJointDof;
			}
			else if (return_state == TARGET_JOINT::ACTIVEJOINT)
			{
				return _activeJointDof;
			}
			else if (return_state == TARGET_JOINT::PASSIVEJOINT)
			{
				return _totalJointDof - _activeJointDof;
			}
			return _totalJointDof;
		}

		void State::writeReturnMatrix(Math::MatrixX& returnMatrix, const Math::MatrixX& value, const unsigned int startRow, const unsigned int jointIndex, const TARGET_JOINT& return_state) const
		{
			unsigned int Idx = _assemIndex[jointIndex];
			if (return_state == TARGET_JOINT::STATEJOINT)
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
			else if (return_state == TARGET_JOINT::ACTIVEJOINT || return_state == TARGET_JOINT::PASSIVEJOINT)
			{
				Idx = _stateIndex[jointIndex];
			}
			returnMatrix.block(startRow, Idx, value.rows(), value.cols()) = value;
		}

		void State::writeReturnVector(Math::VectorX& returnVector, const Math::VectorX& value, const unsigned int jointIndex, const TARGET_JOINT& return_state) const
		{
			unsigned int Idx = _assemIndex[jointIndex];
			if (return_state == TARGET_JOINT::STATEJOINT)
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
			else if (return_state == TARGET_JOINT::ACTIVEJOINT || return_state == TARGET_JOINT::PASSIVEJOINT)
			{
				Idx = _stateIndex[jointIndex];
			}
			returnVector.block(Idx, 0, value.size(), 1) = value;
		}
	}
}
