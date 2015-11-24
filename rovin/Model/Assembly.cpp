#include "Assembly.h"

#include "ScrewJoint.h"

#include <list>
#include <queue>

#include <rovin/utils/Diagnostic.h>

using namespace std;
using namespace rovin::Math;

namespace rovin
{
	namespace Model
	{
		pair< unsigned int, JointDirection > Assembly::reverseDirection(const pair< unsigned int, JointDirection >& target)
		{
			if (target.second == JointDirection::REGULAR)
				return pair< unsigned int, JointDirection >(target.first, JointDirection::REVERSE);
			return pair< unsigned int, JointDirection >(target.first, JointDirection::REVERSE);
		}

		Assembly::Mate::Mate(const Model::JointPtr& joint, const unsigned int mountLinkIdx, const unsigned int actionLinkIdx,
			const SE3& Tmj, const SE3& Tja) : _joint(joint), _mountLinkIdx(mountLinkIdx), _actionLinkIdx(actionLinkIdx),
			_Tmj(Tmj), _Tja(Tja) {}

		unsigned int Assembly::Mate::getParentLinkIdx(const JointDirection& jointDirection) const
		{
			if (jointDirection == JointDirection::REGULAR)
				return _mountLinkIdx;
			else return _actionLinkIdx;
		}

		unsigned int Assembly::Mate::getChildLinkIdx(const JointDirection& jointDirection) const
		{
			if (jointDirection == JointDirection::REGULAR)
				return _actionLinkIdx;
			else return _mountLinkIdx;
		}

		Assembly::Assembly(const Assembly& operand)
		{
			*this = operand;
		}

		Assembly::~Assembly()
		{
			_complete = false;
			_linkPtr.clear();
			_linkIndexMap.clear();
			_mateIndexMap.clear();
			_markerIndexMap.clear();
			_Mate.clear();
			_Tree.clear();
		}

		StatePtr Assembly::makeState() const
		{
			vector< string > linkNameList;
			vector< pair< string, unsigned int >> jointNameList;

			for (unsigned int i = 0; i < _linkPtr.size(); i++)
			{
				linkNameList.push_back(_linkPtr[i]->getName());
			}
			for (unsigned int i = 0; i < _Mate.size(); i++)
			{
				jointNameList.push_back(pair< string, unsigned int>(_Mate[i]._joint->getName(), _Mate[i]._joint->getDOF()));
			}

			return StatePtr(new State(linkNameList, jointNameList));
		}

		Assembly& Assembly::operator = (const Assembly& target)
		{
			if (this != &target)
			{
				_complete = false;
				_linkPtr.clear();
				_linkIndexMap.clear();
				_mateIndexMap.clear();
				_markerIndexMap.clear();
				_Mate.clear();
				_Tree.clear();

				for (unsigned int i = 0; i < target._linkPtr.size(); i++)
				{
					this->addLink(target._linkPtr[i]->copy());
				}
				for (unsigned int i = 0; i < target._Mate.size(); i++)
				{
					this->addMate(target._Mate[i]._joint->copy(), target._linkPtr[target._Mate[i]._mountLinkIdx]->getName(),
						target._linkPtr[target._Mate[i]._actionLinkIdx]->getName(), target._Mate[i]._Tmj, target._Mate[i]._Tja);
				}
			}
			return *this;
		}

		Assembly Assembly::operator + (const Assembly& target)
		{
			Assembly result(this->getName() + "ADD" + target.getName());
			result = (*this);
			result += target;
			return result;
		}

		Assembly& Assembly::operator += (const Assembly& target)
		{
			utils::Log(isCompleted(), "수정하고 싶으면 Assembly 모드로 작업하십시오.", true);
			for (unsigned int i = 0; i < target._linkPtr.size(); i++)
			{
				this->addLink(target._linkPtr[i]->copy());
			}
			for (unsigned int i = 0; i < target._Mate.size(); i++)
			{
				this->addMate(target._Mate[i]._joint->copy(), target._linkPtr[target._Mate[i]._mountLinkIdx]->getName(),
					target._linkPtr[target._Mate[i]._actionLinkIdx]->getName(), target._Mate[i]._Tmj, target._Mate[i]._Tja);
			}
			return *this;
		}

		string Assembly::getName() const
		{
			return _assemblyName;
		}

		void Assembly::setName(const string assemblyName)
		{
			utils::Log(!utils::checkName(assemblyName), "Assembly의 이름으로 설정 할 수 없습니다.", true);
			_assemblyName = assemblyName;
		}

		const vector< LinkPtr >& Assembly::getLinkList() const
		{
			return _linkPtr;
		}

		const vector< Assembly::Mate >& Assembly::getMateList() const
		{
			return _Mate;
		}

		unsigned int Assembly::getLinkIndex(const std::string& linkName) const
		{
			std::map< std::string, unsigned int >::const_iterator iter = _linkIndexMap.find(linkName);
			if (iter == _linkIndexMap.end())
			{
				return -1;
			}
			return iter->second;
		}

		unsigned int Assembly::getLinkIndexByMarkerName(const std::string& markerName) const
		{
			std::map< std::string, unsigned int >::const_iterator iter = _markerIndexMap.find(markerName);
			if (iter == _markerIndexMap.end())
			{
				return -1;
			}
			return iter->second;
		}

		unsigned int Assembly::getMateIndexByJointName(const std::string& jointName) const
		{
			std::map< std::string, unsigned int >::const_iterator iter = _mateIndexMap.find(jointName);
			if (iter == _mateIndexMap.end())
			{
				return -1;
			}
			return iter->second;
		}

		LinkPtr Assembly::getLinkPtr(const std::string& linkName) const
		{
			utils::Log(getLinkIndex(linkName) == -1, "찾고자 하는 Link가 존재하지 않습니다.", true);
			return _linkPtr[getLinkIndex(linkName)];
		}

		JointPtr Assembly::getJointPtr(const std::string& jointName) const
		{
			utils::Log(getMateIndexByJointName(jointName) == -1, "찾고자 하는 Joint가 존재하지 않습니다.", true);
			return _Mate[getMateIndexByJointName(jointName)]._joint;
		}

		LinkPtr Assembly::getLinkPtr(const unsigned int linkIndex) const
		{
			return _linkPtr[linkIndex];
		}

		JointPtr Assembly::getJointPtrByMateIndex(const unsigned int mateIndex) const
		{
			return _Mate[mateIndex]._joint;
		}

		void Assembly::addLink(const Model::LinkPtr& linkPtr)
		{
			utils::Log(isCompleted(), "수정하고 싶으면 Assembly 모드로 작업하십시오.", true);
			utils::Log(getLinkIndex(linkPtr->getName()) != -1 || getLinkIndexByMarkerName(linkPtr->getName()) != -1 || getMateIndexByJointName(linkPtr->getName()) != -1,
				"Link의 이름이 이미 Assembly에 존재합니다.", true);
			std::map< std::string, Math::SE3 > marker = linkPtr->getMarkerMap();
			for (std::map< std::string, Math::SE3 >::const_iterator iter = marker.begin(); iter != marker.end(); iter++)
			{
				utils::Log(getLinkIndex(iter->first) != -1 || getLinkIndexByMarkerName(iter->first) != -1 || getMateIndexByJointName(iter->first) != -1,
					"Marker의 이름이 이미 Assembly에 존재합니다.", true);
			}

			unsigned int Idx = _linkPtr.size();
			_linkPtr.push_back(linkPtr);
			_linkIndexMap.insert(std::pair< std::string, unsigned int >(linkPtr->getName(), Idx));
			for (std::map< std::string, Math::SE3 >::const_iterator iter = marker.begin(); iter != marker.end(); iter++)
			{
				_linkIndexMap.insert(pair< string, unsigned int >(iter->first, Idx));
			}
		}

		void Assembly::addMate(const Model::JointPtr& jointPtr, const Model::LinkPtr& mountLinkPtr, const Model::LinkPtr& actionLinkPtr,
			const Math::SE3& Tmj, const Math::SE3& Tja)
		{
			if (getLinkIndex(mountLinkPtr->getName()) == -1) addLink(mountLinkPtr);
			if (getLinkIndex(actionLinkPtr->getName()) == -1) addLink(actionLinkPtr);

			addMate(jointPtr, mountLinkPtr->getName(), actionLinkPtr->getName(), Tmj, Tja);
		}

		void Assembly::addMate(const Model::JointPtr& jointPtr, const std::string& mountLinkMarkerName, const Model::LinkPtr& actionLinkPtr,
			const Math::SE3& Tmj, const Math::SE3& Tja)
		{
			std::string new_mountLinkMarkerName = mountLinkMarkerName;
			Math::SE3 new_Tmj = Tmj;

			if (getLinkIndex(new_mountLinkMarkerName) != -1);
			else if (getLinkIndexByMarkerName(new_mountLinkMarkerName) != -1)
			{
				unsigned int linkIdx = getLinkIndexByMarkerName(new_mountLinkMarkerName);
				new_Tmj = _linkPtr[linkIdx]->getMarker(new_mountLinkMarkerName) * new_Tmj;
				new_mountLinkMarkerName = _linkPtr[linkIdx]->getName();
			}
			else
			{
				utils::Log("해당하는 Link 또는 Marker가 존재하지 않습니다.", true);
			}
			if (getLinkIndex(actionLinkPtr->getName()) == -1) addLink(actionLinkPtr);

			addMate(jointPtr, new_mountLinkMarkerName, actionLinkPtr->getName(), new_Tmj, Tja);
		}

		void Assembly::addMate(const Model::JointPtr& jointPtr, const Model::LinkPtr& mountLinkPtr, const std::string& actionLinkMarkerName,
			const Math::SE3& Tmj, const Math::SE3& Tja)
		{
			std::string new_actionLinkMarkerName = actionLinkMarkerName;
			Math::SE3 new_Tja = Tja;

			if (getLinkIndex(mountLinkPtr->getName()) == -1) addLink(mountLinkPtr);
			if (getLinkIndex(new_actionLinkMarkerName) != -1);
			else if (getLinkIndexByMarkerName(new_actionLinkMarkerName) != -1)
			{
				unsigned int linkIdx = getLinkIndexByMarkerName(new_actionLinkMarkerName);
				new_Tja = new_Tja * _linkPtr[linkIdx]->getMarker(new_actionLinkMarkerName);
				new_actionLinkMarkerName = _linkPtr[linkIdx]->getName();
			}
			else
			{
				utils::Log("해당하는 Link 또는 Marker가 존재하지 않습니다.", true);
			}

			addMate(jointPtr, mountLinkPtr->getName(), new_actionLinkMarkerName, Tmj, new_Tja);
		}

		void Assembly::addMate(const Model::JointPtr& jointPtr, const std::string& mountLinkMarkerName, const std::string& actionLinkMarkerName,
			const Math::SE3& Tmj, const Math::SE3& Tja)
		{
			utils::Log(isCompleted(), "수정하고 싶으면 Assembly 모드로 작업하십시오.", true);
			utils::Log(getMateIndexByJointName(jointPtr->getName()) != -1 || getLinkIndex(jointPtr->getName()) != -1 || getLinkIndexByMarkerName(jointPtr->getName()) != -1,
				"Joint의 이름이 이미 Assembly에 존재합니다.", true);

			unsigned int mountLinkIdx;
			unsigned int actionLinkIdx;

			Math::SE3 new_Tmj = Tmj;
			Math::SE3 new_Tja = Tja;

			if ((mountLinkIdx = getLinkIndex(mountLinkMarkerName)) != -1);
			else if ((mountLinkIdx = getLinkIndexByMarkerName(mountLinkMarkerName)) != -1)
			{
				new_Tmj = _linkPtr[mountLinkIdx]->getMarker(mountLinkMarkerName) * new_Tmj;
			}
			if ((actionLinkIdx = getLinkIndex(actionLinkMarkerName)) != -1);
			else if ((actionLinkIdx = getLinkIndexByMarkerName(actionLinkMarkerName)) != -1)
			{
				new_Tja = new_Tja * _linkPtr[actionLinkIdx]->getMarker(actionLinkMarkerName);
			}

			unsigned int Idx = _Mate.size();
			_Mate.push_back(Mate(jointPtr, mountLinkIdx, actionLinkIdx, new_Tmj, new_Tja));
			_mateIndexMap.insert(pair< string, unsigned int>(jointPtr->getName(), Idx));
		}

		void Assembly::deleteLink(const string& linkName)
		{
			utils::Log(isCompleted(), "수정하고 싶으면 Assembly 모드로 작업하십시오.", true);

			unsigned int Idx = getLinkIndex(linkName);
			vector< string > nameList;
			for (unsigned int i = 0; i < _Mate.size(); i++)
			{
				if (_Mate[i]._mountLinkIdx == Idx || _Mate[i]._actionLinkIdx == Idx)
				{
					nameList.push_back(_Mate[i]._joint->getName());
				}
			}
			for (unsigned int i = 0; i < nameList.size(); i++)
			{
				deleteMateByJointName(nameList[i]);
			}
			std::map< std::string, Math::SE3 > marker = _linkPtr[Idx]->getMarkerMap();
			for (std::map< std::string, Math::SE3 >::iterator iter = marker.begin(); iter != marker.end(); iter++)
			{
				_markerIndexMap.erase(iter->first);
			}
			_linkPtr.erase(_linkPtr.begin() + Idx);
			_linkIndexMap.clear();
			for (unsigned int i = 0; i < _linkPtr.size(); i++)
			{
				_linkIndexMap.insert(pair< string, unsigned int >(_linkPtr[i]->getName(), i));
			}

			for (unsigned int i = 0; i < _Mate.size(); i++)
			{
				if (_Mate[i]._mountLinkIdx > Idx)
				{
					_Mate[i]._mountLinkIdx--;
				}
				if (_Mate[i]._actionLinkIdx > Idx)
				{
					_Mate[i]._actionLinkIdx--;
				}
			}
		}

		void Assembly::deleteMateByJointName(const string& jointName)
		{
			utils::Log(isCompleted(), "수정하고 싶으면 Assembly 모드로 작업하십시오.", true);

			unsigned int Idx = getMateIndexByJointName(jointName);
			_Mate.erase(_Mate.begin() + Idx);
			_mateIndexMap.clear();
			for (unsigned int i = 0; i < _linkPtr.size(); i++)
			{
				_mateIndexMap.insert(pair< string, unsigned int >(_Mate[i]._joint->getName(), i));
			}
		}

		void Assembly::changeName(const string& oldName, const string& newName)
		{
			utils::Log(isCompleted(), "수정하고 싶으면 Assembly 모드로 작업하십시오.", true);

			if (oldName.compare(newName) == 0) return;

			map< string, unsigned int >::iterator iter = _linkIndexMap.find(oldName);
			if (iter != _linkIndexMap.end())
			{
				_linkPtr[iter->second]->_name = newName;
				_linkIndexMap.insert(pair< string, unsigned int >(newName, iter->second));
				_linkIndexMap.erase(oldName);
			}

			iter = _mateIndexMap.find(oldName);
			if (iter != _mateIndexMap.end())
			{
				_Mate[iter->second]._joint->_name = newName;
				_mateIndexMap.insert(pair< string, unsigned int >(newName, iter->second));
				_mateIndexMap.erase(oldName);
			}

			iter = _markerIndexMap.find(oldName);
			if (iter != _markerIndexMap.end())
			{
				Math::SE3 T = _linkPtr[iter->second]->getMarker(oldName);
				_linkPtr[iter->second]->eraseMarker(oldName);
				_linkPtr[iter->second]->addMarker(newName, T);
				_markerIndexMap.insert(pair< string, unsigned int >(newName, iter->second));
				_linkIndexMap.erase(oldName);
			}
		}

		void Assembly::changeNameAll(const string& prefix)
		{
			utils::Log(isCompleted(), "수정하고 싶으면 Assembly 모드로 작업하십시오.", true);

			vector< string > nameList;
			for (map< string, unsigned int >::iterator iter = _linkIndexMap.begin(); iter != _linkIndexMap.end(); iter++)
			{
				nameList.push_back(iter->first);
			}
			for (map< string, unsigned int >::iterator iter = _mateIndexMap.begin(); iter != _mateIndexMap.end(); iter++)
			{
				nameList.push_back(iter->first);
			}
			for (map< string, unsigned int >::iterator iter = _markerIndexMap.begin(); iter != _markerIndexMap.end(); iter++)
			{
				nameList.push_back(iter->first);
			}
			for (unsigned int i = 0; i < nameList.size(); i++)
			{
				changeName(nameList[i], prefix + nameList[i]);
			}
		}

		AssemblyPtr Assembly::copy(const string& prefix) const
		{
			AssemblyPtr clone = AssemblyPtr(new Assembly("copy" + _assemblyName));
			clone->operator=(*this);
			clone->changeNameAll(prefix);
			return clone;
		}

		bool Assembly::isCompleted() const
		{
			return _complete;
		}

		void Assembly::setAssemblyMode()
		{
			_complete = false;
		}

		void Assembly::completeAssembling(const string& baseLinkName)
		{
			utils::Log(getLinkIndex(baseLinkName) == -1, "입력한 이름을 갖는 Link가 존재하지 않습니다.", true);

			_Tree.clear();
			_Parent.clear();
			_ClosedLoopConstraint.clear();

			_complete = true;
			_baseLink = getLinkIndex(baseLinkName);

			vector< vector< pair< unsigned int, Model::JointDirection >>> adjacencyList(_linkPtr.size());
			for (unsigned int i = 0; i < _Mate.size();i++)
			{
				adjacencyList[_Mate[i]._mountLinkIdx].push_back(pair< unsigned int, Model::JointDirection >(i, JointDirection::REGULAR));
				adjacencyList[_Mate[i]._actionLinkIdx].push_back(pair< unsigned int, Model::JointDirection >(i, JointDirection::REVERSE));
			}

			_Parent.resize(_linkPtr.size());
			_Depth.resize(_linkPtr.size());
			vector< bool > checkLinkVisit(_linkPtr.size()), checkMateVisit(_Mate.size());
			queue< unsigned int > que;
			que.push(_baseLink);
			checkLinkVisit[_baseLink] = true;
			_Depth[_baseLink] = 0;
			while (!que.empty())
			{
				unsigned int currentIdx = que.front();
				que.pop();

				for (unsigned int i = 0; i < adjacencyList[currentIdx].size(); i++)
				{
					unsigned int nextIdx;

					if (checkMateVisit[adjacencyList[currentIdx][i].first]) continue;
					checkMateVisit[adjacencyList[currentIdx][i].first] = true;

					if (adjacencyList[currentIdx][i].second == Model::JointDirection::REGULAR)
					{
						nextIdx = _Mate[adjacencyList[currentIdx][i].first]._actionLinkIdx;
					}
					else
					{
						nextIdx = _Mate[adjacencyList[currentIdx][i].first]._mountLinkIdx;
					}

					if (checkLinkVisit[nextIdx])
					{
						list< pair< unsigned int, Model::JointDirection >> closedloop;
						closedloop.push_back(adjacencyList[currentIdx][i]);
						unsigned int Idx1 = currentIdx;
						unsigned int Idx2 = nextIdx;
						unsigned int depth1 = _Depth[Idx1];
						unsigned int depth2 = _Depth[Idx2];
						while (depth1 > depth2)
						{
							closedloop.push_front(_Parent[Idx1]);
							if (_Parent[Idx1].second == Model::JointDirection::REGULAR)
								Idx1 = _Mate[_Parent[Idx1].first]._mountLinkIdx;
							else Idx1 = _Mate[_Parent[Idx1].first]._actionLinkIdx;
							depth1--;
						}
						while (depth1 < depth2)
						{
							closedloop.push_back(reverseDirection(_Parent[Idx2]));
							if (_Parent[Idx2].second == Model::JointDirection::REGULAR)
								Idx2 = _Mate[_Parent[Idx2].first]._mountLinkIdx;
							else Idx2 = _Mate[_Parent[Idx2].first]._actionLinkIdx;
							depth2--;
						}
						while (Idx1 != Idx2)
						{
							closedloop.push_front(_Parent[Idx1]);
							closedloop.push_back(reverseDirection(_Parent[Idx2]));
							if (_Parent[Idx1].second == Model::JointDirection::REGULAR)
								Idx1 = _Mate[_Parent[Idx1].first]._mountLinkIdx;
							else Idx1 = _Mate[_Parent[Idx1].first]._actionLinkIdx;
							if (_Parent[Idx2].second == Model::JointDirection::REGULAR)
								Idx2 = _Mate[_Parent[Idx2].first]._mountLinkIdx;
							else Idx2 = _Mate[_Parent[Idx2].first]._actionLinkIdx;
							depth1--;
							depth2--;
						}
						_ClosedLoopConstraint.push_back(vector< pair< unsigned int, Model::JointDirection >>());
						for (list< pair< unsigned int, Model::JointDirection >>::iterator iter = closedloop.begin(); iter != closedloop.end(); iter++)
						{
							_ClosedLoopConstraint[_ClosedLoopConstraint.size() - 1].push_back(*iter);
						}
					}
					else
					{
						checkLinkVisit[nextIdx] = true;
						_Depth[nextIdx] = _Depth[currentIdx] + 1;
						_Parent[nextIdx] = adjacencyList[currentIdx][i];
						que.push(nextIdx);
						_Tree.push_back(adjacencyList[currentIdx][i]);
					}
				}
			}
		}

		SE3 Assembly::getTransform(const unsigned int mateIdx, State::JointState& jointState, const JointDirection& jointDirection) const
		{
			if (jointState.getJointReferenceFrame() != JointReferenceFrame::JOINTFRAME)
			{
				jointState.needUpdate(true, true, true);
				jointState.setJointReferenceFrame(JointReferenceFrame::JOINTFRAME);
			}

			if (!jointState.isUpdated(true, false, false))
			{
				_Mate[mateIdx]._joint->updateTransform(jointState);
				jointState.TUpdated();
			}

			if (jointDirection == JointDirection::REGULAR) return _Mate[mateIdx]._Tmj * jointState._T[jointState._dof - 1] * _Mate[mateIdx]._Tja;
			else return (_Mate[mateIdx]._Tmj * jointState._T[jointState._dof - 1] * _Mate[mateIdx]._Tja).inverse();
		}

		Matrix6X Assembly::getJacobian(const unsigned int mateIdx, State::JointState& jointState, const JointDirection& jointDirection) const
		{
			if (jointState.getJointReferenceFrame() != JointReferenceFrame::JOINTFRAME)
			{
				jointState.needUpdate(true, true, true);
				jointState.setJointReferenceFrame(JointReferenceFrame::JOINTFRAME);
			}

			if (!jointState.isUpdated(true, false, false))
			{
				_Mate[mateIdx]._joint->updateTransform(jointState);
				jointState.TUpdated();
			}
			else if (!jointState.isUpdated(false, true, false))
			{
				_Mate[mateIdx]._joint->updateJacobian(jointState);
				jointState.JUpdated();
			}

			if (jointDirection == JointDirection::REGULAR) return SE3::Ad(_Mate[mateIdx]._Tmj)*jointState._J;
			else return (-SE3::InvAd(jointState._T[jointState._dof - 1] * _Mate[mateIdx]._Tja))*jointState._J;
		}

		Matrix6X Assembly::getJacobianDot(const unsigned int mateIdx, State::JointState& jointState, const JointDirection& jointDirection) const
		{
			if (jointState.getJointReferenceFrame() != JointReferenceFrame::JOINTFRAME)
			{
				jointState.needUpdate(true, true, true);
				jointState.setJointReferenceFrame(JointReferenceFrame::JOINTFRAME);
			}

			if (!jointState.isUpdated(true, false, false))
			{
				_Mate[mateIdx]._joint->updateTransform(jointState);
				jointState.TUpdated();
			}
			else if (!jointState.isUpdated(false, true, false))
			{
				_Mate[mateIdx]._joint->updateJacobian(jointState);
				jointState.JUpdated();
			}
			else if (!jointState.isUpdated(false, false, true))
			{
				_Mate[mateIdx]._joint->updateJacobianDot(jointState);
				jointState.JDotUpdated();
			}

			//TODO
			if (jointDirection == JointDirection::REGULAR) return SE3::Ad(_Mate[mateIdx]._Tmj)*jointState._J;
			else return (-SE3::InvAd(jointState._T[0] * _Mate[mateIdx]._Tja))*jointState._J;
		}
	}
}