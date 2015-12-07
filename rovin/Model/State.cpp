#include "State.h"

#include <rovin/utils/Diagnostic.h>

using namespace std;

namespace rovin
{
	namespace Model
	{
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

			_JointReferenceFrame = UNDEFINED;
			//	TODO: set stateInfo to false;
			_stateInfoUpToDate = 0;
		}

		Math::VectorX State::getJointTorque(const TARGET_JOINT& target) const
		{
			Math::VectorX tau(getDOF(target));

			switch (target)
			{
			case TARGET_JOINT::ASSEMJOINT:
				for (unsigned int i = 0; i < _jointState.size(); i++)
					for (unsigned j = 0; j < _jointState[i].getDOF(); j++)
						tau[_assemIndex[i] + j] = _jointState[i]._tau[j];
				break;

			case TARGET_JOINT::STATEJOINT:
				for (unsigned int i = 0; i < _activeJointList.size(); i++)
					for (unsigned int j = 0; j < _jointState[_activeJointList[i]].getDOF(); j++)
						tau[_stateIndex[_activeJointList[i]] + j] = _jointState[_activeJointList[i]]._tau[j];
				for (unsigned int i = 0; i < _passiveJointList.size(); i++)
					for (unsigned int j = 0; j < _jointState[_passiveJointList[i]].getDOF(); j++)
						tau[_activeJointDof + _stateIndex[_passiveJointList[i]] + j] = _jointState[_passiveJointList[i]]._tau[j];
				break;

			case TARGET_JOINT::ACTIVEJOINT:
				for (unsigned int i = 0; i < _activeJointList.size(); i++)
					for (unsigned int j = 0; j < _jointState[_activeJointList[i]].getDOF(); j++)
						tau[_stateIndex[_activeJointList[i]] + j] = _jointState[_activeJointList[i]]._tau[j];
				break;

			case TARGET_JOINT::PASSIVEJOINT:
				for (unsigned int i = 0; i < _passiveJointList.size(); i++)
					for (unsigned int j = 0; j < _jointState[_passiveJointList[i]].getDOF(); j++)
						tau[_stateIndex[_passiveJointList[i]] + j] = _jointState[_passiveJointList[i]]._tau[j];
				break;

			default:
				break;
			}

			return tau;
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

		bool State::getInfoUpToDate(int infoIdx)
		{
			return infoIdx == (infoIdx & _stateInfoUpToDate);
		}

		void State::setInfoUpToDate(int infoIdx, bool upToDate)
		{
			if (upToDate)
				_stateInfoUpToDate |= infoIdx;
			else
				_stateInfoUpToDate &= ~infoIdx;
		}

		State::LinkState& State::getLinkState(const unsigned int linkIndex)
		{
			return _linkState[linkIndex];
		}

		State::LinkState& State::getLinkState(const string& linkName)
		{
			return _linkState[getLinkIndex(linkName)];
		}

		const State::LinkState& State::getLinkState(const unsigned int linkIndex) const
		{
			return _linkState[linkIndex];
		}

		const State::LinkState& State::getLinkState(const string& linkName) const
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

		State::JointState & State::getJointState(const TARGET_JOINT & target, const unsigned int idx)
		{
			switch (target)
			{
			case ASSEMJOINT:
				return _jointState[idx];
			default:
			case STATEJOINT:
				if (idx < _activeJointList.size())
					return _jointState[_activeJointList[idx]];
				else
					return _jointState[_passiveJointList[idx - _activeJointList.size()]];
			case ACTIVEJOINT:
				return _jointState[_activeJointList[idx]];
			case PASSIVEJOINT:
				return _jointState[_passiveJointList[idx]];

			}
		}

		const State::JointState& State::getJointState(const unsigned int jointIndex) const
		{
			return _jointState[jointIndex];
		}

		const State::JointState& State::getJointStateByMateIndex(const unsigned int mateIndex) const
		{
			return _jointState[mateIndex];
		}

		const Math::VectorX & State::getJointq(const JointState & jointState) const
		{
			return jointState.getq();
		}

		const Math::VectorX & State::getJointq(const unsigned int jointIdx) const
		{
			return getJointq(_jointState[jointIdx]);
		}

		const Math::VectorX & State::getJointq(const std::string & jointName) const
		{
			return getJointq(getJointState(jointName));
		}

		const State::JointState& State::getJointState(const string& jointName) const
		{
			return _jointState[getJointIndex(jointName)];
		}

		const State::JointState& State::getJointState(const TARGET_JOINT& target, const unsigned int idx) const
		{
			switch (target)
			{
			case ASSEMJOINT:
				return _jointState[idx];
			default:
			case STATEJOINT:
				if (idx < _activeJointList.size())
					return _jointState[_activeJointList[idx]];
				else
					return _jointState[_passiveJointList[idx - _activeJointList.size()]];
			case ACTIVEJOINT:
				return _jointState[_activeJointList[idx]];
			case PASSIVEJOINT:
				return _jointState[_passiveJointList[idx]];
			}
		}

		unsigned int State::getLinkIndex(const string& linkName) const
		{
			return _linkIndexMap.find(linkName)->second;
		}

		std::vector<std::string> State::getJointList(const TARGET_JOINT & target) const
		{
			vector< string > jointNameList(getDOF(target));
			switch (target)
			{
			case ASSEMJOINT:
				for (unsigned int i = 0; i < jointNameList.size(); i++)
					jointNameList[i] = _jointName[i];
			default:
			case STATEJOINT:
				for (unsigned int i = 0; i < jointNameList.size(); i++)
				{
					if (i < _activeJointList.size())
						jointNameList[i] = _jointName[_activeJointList[i]];
					else
						jointNameList[i] = _jointName[_passiveJointList[i - _activeJointList.size()]];
				}
			case ACTIVEJOINT:
				for (unsigned int i = 0; i < jointNameList.size(); i++)
					jointNameList[i] = _jointName[_activeJointList[i]];
			case PASSIVEJOINT:
				for (unsigned int i = 0; i < jointNameList.size(); i++)
					jointNameList[i] = _jointName[_passiveJointList[i]];
			}
			return jointNameList;
		}

		unsigned int State::getJointID(const TARGET_JOINT& target, const unsigned int idx) const
		{
			unsigned int ID;

			switch (target)
			{
			case ASSEMJOINT:
				ID = idx;
			default:
			case STATEJOINT:
				if (idx < _activeJointList.size())
					ID = _activeJointList[idx];
				else ID = _passiveJointList[idx - _activeJointList.size()];
			case ACTIVEJOINT:
				ID = _activeJointList[idx];
			case PASSIVEJOINT:
				ID = _passiveJointList[idx];
			}

			return ID;
		}

		unsigned int State::getJointIndex(const string& jointName) const
		{
			return _jointIndexMap.find(jointName)->second;
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
					_stateIndex[_passiveJointList[i]] -= _jointState[_passiveJointList[Idx]].getDOF();
				}
				if (_jointName[_passiveJointList[i]].compare(jointName) == 0)
				{
					Idx = i;
					flag = true;

					utils::Log(_jointState[_passiveJointList[Idx]].getDOF() == 0, "Active joint의 Dof는 0이면 안됩니다.", true);
				}
			}

			_isActiveJoint[_passiveJointList[Idx]] = true;
			_activeJointList.push_back(_passiveJointList[Idx]);
			_stateIndex[_passiveJointList[Idx]] = _activeJointDof;
			_activeJointDof += _jointState[_passiveJointList[Idx]].getDOF();

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
					_stateIndex[_activeJointList[i]] -= _jointState[_activeJointList[Idx]].getDOF();
				}
				if (_jointName[_activeJointList[i]].compare(jointName) == 0)
				{
					Idx = i;
					flag = true;
				}
			}

			_passiveJointList.push_back(_activeJointList[Idx]);
			_stateIndex[_activeJointList[Idx]] = _totalJointDof - _activeJointDof;
			_activeJointDof -= _jointState[_activeJointList[Idx]].getDOF();

			_isActiveJoint[_activeJointList[Idx]] = false;
			_activeJointList.erase(_activeJointList.begin() + Idx);
		}

		Math::VectorX State::getJointq(const TARGET_JOINT & target) const
		{
			Math::VectorX q(getDOF(target));
			switch (target)
			{
			case TARGET_JOINT::ASSEMJOINT:
				for (unsigned int i = 0; i < _jointState.size(); i++)
					for (unsigned j = 0; j < _jointState[i].getDOF(); j++)
						q[_assemIndex[i] + j] = _jointState[i].getq(j);
				break;

			case TARGET_JOINT::STATEJOINT:
				for (unsigned int i = 0; i < _activeJointList.size(); i++)
					for (unsigned int j = 0; j < _jointState[_activeJointList[i]].getDOF(); j++)
						q[_stateIndex[_activeJointList[i]] + j] = _jointState[_activeJointList[i]].getq(j);
				for (unsigned int i = 0; i < _passiveJointList.size(); i++)
					for (unsigned int j = 0; j < _jointState[_passiveJointList[i]].getDOF(); j++)
						q[_activeJointDof + _stateIndex[_passiveJointList[i]] + j] = _jointState[_passiveJointList[i]].getq(j);
				break;

			case TARGET_JOINT::ACTIVEJOINT:
				for (unsigned int i = 0; i < _activeJointList.size(); i++)
					for (unsigned int j = 0; j < _jointState[_activeJointList[i]].getDOF(); j++)
						q[_stateIndex[_activeJointList[i]] + j] = _jointState[_activeJointList[i]].getq(j);
				break;

			case TARGET_JOINT::PASSIVEJOINT:
				for (unsigned int i = 0; i < _passiveJointList.size(); i++)
					for (unsigned int j = 0; j < _jointState[_passiveJointList[i]].getDOF(); j++)
						q[_stateIndex[_passiveJointList[i]] + j] = _jointState[_passiveJointList[i]].getq(j);
				break;

			default:
				break;
			}

			return q;
		}

		const Math::VectorX & State::getJointq(const TARGET_JOINT & target, unsigned int jointIdx) const
		{
			return getJointState(target, jointIdx).getq();
		}

		Math::VectorX State::getJointq(const TARGET_JOINT & target, const Math::VectorU& jointIndices) const
		{
			Math::VectorX ret(getDOF(target));
			unsigned int dofIdx = 0;
			for (int i = 0; i < jointIndices.size(); i++)
			{
				const JointState& jStat = getJointState(target, jointIndices[i]);
				ret.segment(dofIdx, jStat.getDOF()) = jStat.getq();
				dofIdx += jStat.getDOF();
			}
			ret.conservativeResize(dofIdx);
			return ret;
		}

		const Math::VectorX & State::getJointqdot(const unsigned int jointIdx) const
		{
			return getJointqdot(_jointState[jointIdx]);
		}

		const Math::VectorX & State::getJointqdot(const std::string & jointName) const
		{
			return getJointqdot(getJointState(jointName));
		}

		const Math::VectorX & State::getJointqdot(const JointState & jointState) const
		{
			return jointState.getqdot();
		}

		void State::setJointq(JointState & jointState, const Math::VectorX & q)
		{
			jointState.setq(q); setInfoUpToDate(LINKS_POS | LINKS_VEL | LINKS_ACC | JOINTS_T_FROM_BASE | JOINTS_JACOBIAN | JOINTS_JACOBIAN_DOT, false);
		}

		void State::setJointq(const unsigned int jointIdx, const Math::VectorX & q)
		{
			setJointq(_jointState[jointIdx], q);
		}

		void State::setJointq(const std::string & jointName, const Math::VectorX & q)
		{
			setJointq(getJointState(jointName), q);
		}

		void State::setJointq(const TARGET_JOINT & target, const Math::VectorX & q)
		{
			switch (target)
			{
			case TARGET_JOINT::ASSEMJOINT:
				for (unsigned int i = 0; i < _jointState.size(); i++)
					setJointq(_jointState[i], q.segment(_assemIndex[i], _jointState[i].getDOF()));
				break;
			case TARGET_JOINT::STATEJOINT:
				for (unsigned int i = 0; i < _activeJointList.size(); i++)
					setJointq(_jointState[_activeJointList[i]], q.segment(_stateIndex[_activeJointList[i]], _jointState[_activeJointList[i]].getDOF()));
				for (unsigned int i = 0; i < _passiveJointList.size(); i++)
					setJointq(_jointState[_passiveJointList[i]], q.segment(_activeJointDof + _stateIndex[_passiveJointList[i]], _jointState[_passiveJointList[i]].getDOF()));
				break;
			case TARGET_JOINT::ACTIVEJOINT:
				for (unsigned int i = 0; i < _activeJointList.size(); i++)
					setJointq(_jointState[_activeJointList[i]], q.segment(_stateIndex[_activeJointList[i]], _jointState[_activeJointList[i]].getDOF()));
				break;

			case TARGET_JOINT::PASSIVEJOINT:
				for (unsigned int i = 0; i < _passiveJointList.size(); i++)
					setJointq(_jointState[_passiveJointList[i]], q.segment(_stateIndex[_passiveJointList[i]], _jointState[_passiveJointList[i]].getDOF()));
				break;

			default:
				break;
			}
		}

		void State::setJointq(const TARGET_JOINT & target, unsigned int jointIdx, const Math::VectorX & q)
		{
			switch (target)
			{
			case TARGET_JOINT::ASSEMJOINT:
				setJointq(_jointState[jointIdx], q);
				break;
			case TARGET_JOINT::STATEJOINT:
				if (jointIdx < _activeJointDof)
					setJointq(_jointState[_activeJointList[jointIdx]], q);
				else
					setJointq(_jointState[_passiveJointList[jointIdx]], q);
				break;
			case TARGET_JOINT::ACTIVEJOINT:
				setJointq(_jointState[_activeJointList[jointIdx]], q);
				break;

			case TARGET_JOINT::PASSIVEJOINT:
				setJointq(_jointState[_passiveJointList[jointIdx]], q);
				break;
			default:
				break;
			}
		}

		void State::setJointq(const TARGET_JOINT & target, const Math::VectorU& jointIndices, const Math::VectorX & q)
		{
			unsigned int qIdx = 0;
			for (int i = 0; i < jointIndices.size(); i++)
			{
				const JointState& jStat = getJointState(target, jointIndices[i]);
				setJointq(target, jointIndices[i], q.segment(qIdx, jStat.getDOF()));
				qIdx += jStat.getDOF();
			}
		}

		void State::setJointqdot(JointState & jointState, const Math::VectorX & qdot)
		{
			jointState.setqdot(qdot);
			setInfoUpToDate(JOINTS_JACOBIAN_DOT | LINKS_VEL | LINKS_ACC, false);
		}

		void State::setJointqdot(const unsigned int jointIdx, const Math::VectorX & qdot)
		{
			setJointqdot(_jointState[jointIdx], qdot);
		}

		void State::setJointqdot(const std::string & jointName, const Math::VectorX & qdot)
		{
			setJointqdot(getJointState(jointName), qdot);
		}

		Math::VectorX State::getJointqdot(const TARGET_JOINT & target) const
		{
			Math::VectorX dq(getDOF(target));
			switch (target)
			{
			case TARGET_JOINT::ASSEMJOINT:
				for (unsigned int i = 0; i < _jointState.size(); i++)
					for (unsigned j = 0; j < _jointState[i].getDOF(); j++)
						dq[_assemIndex[i] + j] = _jointState[i].getqdot(j);
				break;

			case TARGET_JOINT::STATEJOINT:
				for (unsigned int i = 0; i < _activeJointList.size(); i++)
					for (unsigned int j = 0; j < _jointState[_activeJointList[i]].getDOF(); j++)
						dq[_stateIndex[_activeJointList[i]] + j] = _jointState[_activeJointList[i]].getqdot(j);
				for (unsigned int i = 0; i < _passiveJointList.size(); i++)
					for (unsigned int j = 0; j < _jointState[_passiveJointList[i]].getDOF(); j++)
						dq[_activeJointDof + _stateIndex[_passiveJointList[i]] + j] = _jointState[_passiveJointList[i]].getqdot(j);
				break;

			case TARGET_JOINT::ACTIVEJOINT:
				for (unsigned int i = 0; i < _activeJointList.size(); i++)
					for (unsigned int j = 0; j < _jointState[_activeJointList[i]].getDOF(); j++)
						dq[_stateIndex[_activeJointList[i]] + j] = _jointState[_activeJointList[i]].getqdot(j);
				break;

			case TARGET_JOINT::PASSIVEJOINT:
				for (unsigned int i = 0; i < _passiveJointList.size(); i++)
					for (unsigned int j = 0; j < _jointState[_passiveJointList[i]].getDOF(); j++)
						dq[_stateIndex[_passiveJointList[i]] + j] = _jointState[_passiveJointList[i]].getqdot(j);
				break;

			default:
				break;
			}

			return dq;
		}

		const Math::VectorX & State::getJointqdot(const TARGET_JOINT & target, unsigned int jointIdx) const
		{
			return getJointState(target, jointIdx).getqdot();
		}

		Math::VectorX State::getJointqdot(const TARGET_JOINT & target, const Math::VectorU & jointIndices) const
		{
			Math::VectorX ret(getDOF(target));
			unsigned int dofIdx = 0;
			for (int i = 0; i < jointIndices.size(); i++)
			{
				const JointState& jStat = getJointState(target, jointIndices[i]);
				ret.segment(dofIdx, jStat.getDOF()) = jStat.getqdot();
				dofIdx += jStat.getDOF();
			}
			ret.conservativeResize(dofIdx);
			return ret;
		}

		const Math::VectorX & State::getJointqddot(const JointState & jointState) const
		{
			return jointState.getqddot();
		}

		const Math::VectorX & State::getJointqddot(const unsigned int jointIdx) const
		{
			return getJointqddot(_jointState[jointIdx]);
		}

		const Math::VectorX & State::getJointqddot(const std::string & jointName) const
		{
			return getJointqddot(getJointState(jointName));
		}


		void State::setJointqdot(const TARGET_JOINT & target, const Math::VectorX & qdot)
		{
			switch (target)
			{
			case TARGET_JOINT::ASSEMJOINT:
				for (unsigned int i = 0; i < _jointState.size(); i++)
					setJointqdot(_jointState[i], qdot.segment(_assemIndex[i], _jointState[i].getDOF()));
				break;
			case TARGET_JOINT::STATEJOINT:
				for (unsigned int i = 0; i < _activeJointList.size(); i++)
					setJointqdot(_jointState[_activeJointList[i]], qdot.segment(_stateIndex[_activeJointList[i]], _jointState[_activeJointList[i]].getDOF()));
				for (unsigned int i = 0; i < _passiveJointList.size(); i++)
					setJointqdot(_jointState[_passiveJointList[i]], qdot.segment(_activeJointDof + _stateIndex[_passiveJointList[i]], _jointState[_passiveJointList[i]].getDOF()));
				break;
			case TARGET_JOINT::ACTIVEJOINT:
				for (unsigned int i = 0; i < _activeJointList.size(); i++)
					setJointqdot(_jointState[_activeJointList[i]], qdot.segment(_stateIndex[_activeJointList[i]], _jointState[_activeJointList[i]].getDOF()));
				break;

			case TARGET_JOINT::PASSIVEJOINT:
				for (unsigned int i = 0; i < _passiveJointList.size(); i++)
					setJointqdot(_jointState[_passiveJointList[i]], qdot.segment(_stateIndex[_passiveJointList[i]], _jointState[_passiveJointList[i]].getDOF()));
				break;

			default:
				break;
			}
		}

		void State::setJointqdot(const TARGET_JOINT & target, unsigned int jointIdx, const Math::VectorX & qdot)
		{
			switch (target)
			{
			case TARGET_JOINT::ASSEMJOINT:
				setJointqdot(_jointState[jointIdx], qdot);
				break;
			case TARGET_JOINT::STATEJOINT:
				if (jointIdx < _activeJointDof)
					setJointqdot(_jointState[_activeJointList[jointIdx]], qdot);
				else
					setJointqdot(_jointState[_passiveJointList[jointIdx]], qdot);
				break;
			case TARGET_JOINT::ACTIVEJOINT:
				setJointqdot(_jointState[_activeJointList[jointIdx]], qdot);
				break;

			case TARGET_JOINT::PASSIVEJOINT:
				setJointqdot(_jointState[_passiveJointList[jointIdx]], qdot);
				break;
			default:
				break;
			}
		}
		void State::setJointqdot(const TARGET_JOINT & target, const Math::VectorU & jointIndices, const Math::VectorX & qdot)
		{
			unsigned int dofIdx = 0;
			for (int i = 0; i < jointIndices.size(); i++)
			{
				const JointState& jStat = getJointState(target, jointIndices[i]);
				setJointqdot(target, jointIndices[i], qdot.segment(dofIdx, jStat.getDOF()));
				dofIdx += jStat.getDOF();
			}
		}

		void State::setJointqddot(JointState & jointState, const Math::VectorX & qddot)
		{
			jointState.setqddot(qddot);
			setInfoUpToDate(LINKS_ACC, false);
		}

		void State::setJointqddot(const unsigned int jointIdx, const Math::VectorX & qddot)
		{
			setJointqddot(_jointState[jointIdx], qddot);
		}

		void State::setJointqddot(const std::string & jointName, const Math::VectorX & qddot)
		{
			setJointqddot(getJointState(jointName), qddot);
		}

		Math::VectorX State::getJointqddot(const TARGET_JOINT & target) const
		{
			Math::VectorX ddq(getDOF(target));
			switch (target)
			{
			case TARGET_JOINT::ASSEMJOINT:
				for (unsigned int i = 0; i < _jointState.size(); i++)
					for (unsigned j = 0; j < _jointState[i].getDOF(); j++)
						ddq[_assemIndex[i] + j] = _jointState[i].getqddot(j);
				break;

			case TARGET_JOINT::STATEJOINT:
				for (unsigned int i = 0; i < _activeJointList.size(); i++)
					for (unsigned int j = 0; j < _jointState[_activeJointList[i]].getDOF(); j++)
						ddq[_stateIndex[_activeJointList[i]] + j] = _jointState[_activeJointList[i]].getqddot(j);
				for (unsigned int i = 0; i < _passiveJointList.size(); i++)
					for (unsigned int j = 0; j < _jointState[_passiveJointList[i]].getDOF(); j++)
						ddq[_activeJointDof + _stateIndex[_passiveJointList[i]] + j] = _jointState[_passiveJointList[i]].getqddot(j);
				break;

			case TARGET_JOINT::ACTIVEJOINT:
				for (unsigned int i = 0; i < _activeJointList.size(); i++)
					for (unsigned int j = 0; j < _jointState[_activeJointList[i]].getDOF(); j++)
						ddq[_stateIndex[_activeJointList[i]] + j] = _jointState[_activeJointList[i]].getqddot(j);
				break;

			case TARGET_JOINT::PASSIVEJOINT:
				for (unsigned int i = 0; i < _passiveJointList.size(); i++)
					for (unsigned int j = 0; j < _jointState[_passiveJointList[i]].getDOF(); j++)
						ddq[_stateIndex[_passiveJointList[i]] + j] = _jointState[_passiveJointList[i]].getqddot(j);
				break;

			default:
				break;
			}

			return ddq;
		}

		const Math::VectorX & State::getJointqddot(const TARGET_JOINT & target, unsigned int jointIdx) const
		{
			return getJointState(target, jointIdx).getqddot();
		}

		Math::VectorX State::getJointqddot(const TARGET_JOINT & target, const Math::VectorU & jointIndices) const
		{
			Math::VectorX ret(getDOF(target));
			unsigned int dofIdx = 0;
			for (int i = 0; i < jointIndices.size(); i++)
			{
				const JointState& jStat = getJointState(target, jointIndices[i]);
				ret.segment(dofIdx, jStat.getDOF()) = jStat.getqddot();
				dofIdx += jStat.getDOF();
			}
			ret.conservativeResize(dofIdx);
			return ret;
		}

		void State::setJointqddot(const TARGET_JOINT & target, const Math::VectorX & qddot)
		{
			switch (target)
			{
			case TARGET_JOINT::ASSEMJOINT:
				for (unsigned int i = 0; i < _jointState.size(); i++)
					setJointqddot(_jointState[i], qddot.segment(_assemIndex[i], _jointState[i].getDOF()));
				break;
			case TARGET_JOINT::STATEJOINT:
				for (unsigned int i = 0; i < _activeJointList.size(); i++)
					setJointqddot(_jointState[_activeJointList[i]], qddot.segment(_stateIndex[_activeJointList[i]], _jointState[_activeJointList[i]].getDOF()));
				for (unsigned int i = 0; i < _passiveJointList.size(); i++)
					setJointqddot(_jointState[_passiveJointList[i]], qddot.segment(_activeJointDof + _stateIndex[_passiveJointList[i]], _jointState[_passiveJointList[i]].getDOF()));
				break;
			case TARGET_JOINT::ACTIVEJOINT:
				for (unsigned int i = 0; i < _activeJointList.size(); i++)
					setJointqddot(_jointState[_activeJointList[i]], qddot.segment(_stateIndex[_activeJointList[i]], _jointState[_activeJointList[i]].getDOF()));
				break;

			case TARGET_JOINT::PASSIVEJOINT:
				for (unsigned int i = 0; i < _passiveJointList.size(); i++)
					setJointqddot(_jointState[_passiveJointList[i]], qddot.segment(_stateIndex[_passiveJointList[i]], _jointState[_passiveJointList[i]].getDOF()));
				break;

			default:
				break;
			}
		}

		void State::setJointqddot(const TARGET_JOINT & target, unsigned int jointIdx, const Math::VectorX & qddot)
		{
			switch (target)
			{
			case TARGET_JOINT::ASSEMJOINT:
				setJointqddot(_jointState[jointIdx], qddot);
				break;
			case TARGET_JOINT::STATEJOINT:
				if (jointIdx < _activeJointDof)
					setJointqddot(_jointState[_activeJointList[jointIdx]], qddot);
				else
					setJointqddot(_jointState[_passiveJointList[jointIdx]], qddot);
				break;
			case TARGET_JOINT::ACTIVEJOINT:
				setJointqddot(_jointState[_activeJointList[jointIdx]], qddot);
				break;

			case TARGET_JOINT::PASSIVEJOINT:
				setJointqddot(_jointState[_passiveJointList[jointIdx]], qddot);
				break;
			default:
				break;
			}
		}

		void State::setJointqddot(const TARGET_JOINT & target, const Math::VectorU & jointIndices, const Math::VectorX & qddot)
		{
			unsigned int dofIdx = 0;
			for (int i = 0; i < jointIndices.size(); i++)
			{
				const JointState& jStat = getJointState(target, jointIndices[i]);
				setJointqddot(target, jointIndices[i], qddot.segment(dofIdx, jStat.getDOF()));
				dofIdx += jStat.getDOF();
			}
		}

		void State::addJointq(const TARGET_JOINT & target, const Math::VectorX & q)
		{
			switch (target)
			{
			case TARGET_JOINT::ASSEMJOINT:
				for (unsigned int i = 0; i < _jointState.size(); i++)
					_jointState[i].addq(q.segment(_assemIndex[i], _jointState[i].getDOF()));
				break;
			case TARGET_JOINT::STATEJOINT:
				for (unsigned int i = 0; i < _activeJointList.size(); i++)
					_jointState[_activeJointList[i]].addq(q.segment(_stateIndex[_activeJointList[i]], _jointState[_activeJointList[i]].getDOF()));
				for (unsigned int i = 0; i < _passiveJointList.size(); i++)
					_jointState[_passiveJointList[i]].addq(q.segment(_activeJointDof + _stateIndex[_passiveJointList[i]], _jointState[_passiveJointList[i]].getDOF()));
				break;
			case TARGET_JOINT::ACTIVEJOINT:
				for (unsigned int i = 0; i < _activeJointList.size(); i++)
					_jointState[_activeJointList[i]].addq(q.segment(_stateIndex[_activeJointList[i]], _jointState[_activeJointList[i]].getDOF()));
				break;
			case TARGET_JOINT::PASSIVEJOINT:
				for (unsigned int i = 0; i < _passiveJointList.size(); i++)
					_jointState[_passiveJointList[i]].addq(q.segment(_stateIndex[_passiveJointList[i]], _jointState[_passiveJointList[i]].getDOF()));
				break;

			default:
				break;
			}
			setInfoUpToDate(JOINTS_T_FROM_BASE | JOINTS_JACOBIAN | JOINTS_JACOBIAN_DOT | LINKS_POS | LINKS_VEL | LINKS_ACC, false);
		}

		void State::addJointqdot(const TARGET_JOINT & target, const Math::VectorX & qdot)
		{
			switch (target)
			{
			case TARGET_JOINT::ASSEMJOINT:
				for (unsigned int i = 0; i < _jointState.size(); i++)
					_jointState[i].addqdot(qdot.segment(_assemIndex[i], _jointState[i].getDOF()));
				break;
			case TARGET_JOINT::STATEJOINT:
				for (unsigned int i = 0; i < _activeJointList.size(); i++)
					_jointState[_activeJointList[i]].addqdot(qdot.segment(_stateIndex[_activeJointList[i]], _jointState[_activeJointList[i]].getDOF()));
				for (unsigned int i = 0; i < _passiveJointList.size(); i++)
					_jointState[_passiveJointList[i]].addqdot(qdot.segment(_activeJointDof + _stateIndex[_passiveJointList[i]], _jointState[_passiveJointList[i]].getDOF()));
				break;
			case TARGET_JOINT::ACTIVEJOINT:
				for (unsigned int i = 0; i < _activeJointList.size(); i++)
					_jointState[_activeJointList[i]].addqdot(qdot.segment(_stateIndex[_activeJointList[i]], _jointState[_activeJointList[i]].getDOF()));
				break;
			case TARGET_JOINT::PASSIVEJOINT:
				for (unsigned int i = 0; i < _passiveJointList.size(); i++)
					_jointState[_passiveJointList[i]].addqdot(qdot.segment(_stateIndex[_passiveJointList[i]], _jointState[_passiveJointList[i]].getDOF()));
				break;

			default:
				break;
			}
			setInfoUpToDate(JOINTS_JACOBIAN_DOT | LINKS_VEL | LINKS_ACC, false);
		}

		void State::addJointqddot(const TARGET_JOINT & target, const Math::VectorX & qddot)
		{
			switch (target)
			{
			case TARGET_JOINT::ASSEMJOINT:
				for (unsigned int i = 0; i < _jointState.size(); i++)
					_jointState[i].addqddot(qddot.segment(_assemIndex[i], _jointState[i].getDOF()));
				break;
			case TARGET_JOINT::STATEJOINT:
				for (unsigned int i = 0; i < _activeJointList.size(); i++)
					_jointState[_activeJointList[i]].addqddot(qddot.segment(_stateIndex[_activeJointList[i]], _jointState[_activeJointList[i]].getDOF()));
				for (unsigned int i = 0; i < _passiveJointList.size(); i++)
					_jointState[_passiveJointList[i]].addqddot(qddot.segment(_activeJointDof + _stateIndex[_passiveJointList[i]], _jointState[_passiveJointList[i]].getDOF()));
				break;
			case TARGET_JOINT::ACTIVEJOINT:
				for (unsigned int i = 0; i < _activeJointList.size(); i++)
					_jointState[_activeJointList[i]].addqddot(qddot.segment(_stateIndex[_activeJointList[i]], _jointState[_activeJointList[i]].getDOF()));
				break;
			case TARGET_JOINT::PASSIVEJOINT:
				for (unsigned int i = 0; i < _passiveJointList.size(); i++)
					_jointState[_passiveJointList[i]].addqddot(qddot.segment(_stateIndex[_passiveJointList[i]], _jointState[_passiveJointList[i]].getDOF()));
				break;

			default:
				break;
			}
			setInfoUpToDate(LINKS_ACC, false);
		}

		void State::addJointTorque(const TARGET_JOINT & target, const Math::VectorX & tau)
		{
			switch (target)
			{
			case TARGET_JOINT::ASSEMJOINT:
				for (unsigned int i = 0; i < _jointState.size(); i++)
					_jointState[i]._tau = tau.segment(_assemIndex[i], _jointState[i].getDOF());
				break;
			case TARGET_JOINT::STATEJOINT:
				for (unsigned int i = 0; i < _activeJointList.size(); i++)
					_jointState[_activeJointList[i]]._tau = tau.segment(_stateIndex[_activeJointList[i]], _jointState[_activeJointList[i]].getDOF());
				for (unsigned int i = 0; i < _passiveJointList.size(); i++)
					_jointState[_passiveJointList[i]]._tau = tau.segment(_activeJointDof + _stateIndex[_passiveJointList[i]], _jointState[_passiveJointList[i]].getDOF());
				break;
			case TARGET_JOINT::ACTIVEJOINT:
				for (unsigned int i = 0; i < _activeJointList.size(); i++)
					_jointState[_activeJointList[i]]._tau = tau.segment(_stateIndex[_activeJointList[i]], _jointState[_activeJointList[i]].getDOF());
				break;
			case TARGET_JOINT::PASSIVEJOINT:
				for (unsigned int i = 0; i < _passiveJointList.size(); i++)
					_jointState[_passiveJointList[i]]._tau = tau.segment(_stateIndex[_passiveJointList[i]], _jointState[_passiveJointList[i]].getDOF());
				break;

			default:
				break;
			}
		}

		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
		State::JointState::JointState(const unsigned int & dof)
			:_dof(dof)
		{
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

				_JointReferenceFrame = UNDEFINED;
			}
			else
			{
				setInfoUpToDate(ALL_INFO, true);
			}

			_constraintF.setZero();

		}
		bool State::JointState::getInfoUpToDate(int infoIdx)
		{
			return infoIdx == (infoIdx & _jointInfoUpToDate);;
		}
		void State::JointState::setInfoUpToDate(int infoIdx, bool upToDate)
		{
			if (upToDate)
				_jointInfoUpToDate |= infoIdx;
			else
				_jointInfoUpToDate &= ~infoIdx;
		}
	}
}
