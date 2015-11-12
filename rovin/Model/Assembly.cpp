#include "Assembly.h"

#include <list>

#include <rovin/utils/Diagnostic.h>

using namespace std;

namespace rovin
{
	namespace Model
	{
		Assembly::Assembly(const Assembly& operand)
		{
			*this = operand;
		}

		Assembly::~Assembly()
		{
			_link.clear();
			_joint.clear();
			_marker.clear();
			_connection.clear();
		}

		Assembly& Assembly::operator = (const Assembly& operand)
		{
			if (this != &operand)
			{
				_link.clear();
				_joint.clear();
				_marker.clear();
				_connection.clear();

				_lock = 0;

				for (map< string, shared_ptr<Link> >::const_iterator iter = operand._link.begin(); iter != operand._link.end(); iter++)
				{
					this->addLink(iter->second->copy());
				}
				for (list< Connection >::const_iterator iter = operand._connection.begin(); iter != operand._connection.end(); iter++)
				{
					this->addJoint(iter->_joint_pointer->copy(), iter->_mountLink_pointer->getName(), iter->_actionLink_pointer->getName(),
						iter->_mountLink_T, iter->_actionLink_T);
				}
			}
			return *this;
		}

		Assembly Assembly::operator + (const Assembly& operand)
		{
			Assembly result;

			result = *this;
			result += operand;

			return result;
		}

		Assembly& Assembly::operator += (const Assembly& operand)
		{
			utils::Log(_lock > 0, "이 어셈블리는 이미 System에 들어가있습니다. 구조를 수정할 수 없습니다.", true);

			for (map< string, shared_ptr<Link> >::const_iterator iter = operand._link.begin(); iter != operand._link.end(); iter++)
			{
				this->addLink(iter->second->copy());
			}
			for (list< Connection >::const_iterator iter = operand._connection.begin(); iter != operand._connection.end(); iter++)
			{
				this->addJoint(iter->_joint_pointer->copy(), iter->_mountLink_pointer->getName(), iter->_actionLink_pointer->getName(),
					iter->_mountLink_T, iter->_actionLink_T);
			}

			return *this;
		}

		void Assembly::LOCK()
		{
			_lock++;
		}

		void Assembly::UNLOCK()
		{
			_lock--;
		}

		shared_ptr<Link>& Assembly::addLink(const shared_ptr<Link>& link_pointer)
		{
			utils::Log(_lock > 0, "이 어셈블리는 이미 System에 들어가있습니다. 구조를 수정할 수 없습니다.", true);

			map< string, shared_ptr<Link> >::iterator iter = _link.find(link_pointer->getName());
			if (iter != _link.end())
			{
				assert(iter->second == link_pointer && "중복되는 이름을 갖는 링크가 이미 어셈블리에 존재합니다");
			}
			else
			{
				assert(!findNameList(link_pointer->getName()) && "중복되는 이름이 이미 어셈블리에 존재합니다");
				map< string, Math::SE3 >  marker_map = link_pointer->getMarkerMap();
				for (map< string, Math::SE3 >::iterator miter = marker_map.begin(); miter != marker_map.end(); miter++)
				{
					assert(!findNameList(miter->first) && "링크에 포함되어 있는 마커의 이름 중에 어셈블리에 이미 존재하는 이름이 있습니다.");
				}

				_link.insert(pair< string, shared_ptr<Link> >(link_pointer->getName(), link_pointer));
				iter = _link.find(link_pointer->getName());

				for (map< string, Math::SE3 >::iterator miter = marker_map.begin(); miter != marker_map.end(); miter++)
				{
					_marker.insert(pair< string, shared_ptr<Link> >(miter->first, iter->second));
				}
			}
			return iter->second;
		}

		void Assembly::deleteLink(const std::shared_ptr<Link>& link_pointer)
		{
			utils::Log(_lock > 0, "이 어셈블리는 이미 System에 들어가있습니다. 구조를 수정할 수 없습니다.", true);

			assert(isLink(link_pointer->getName()) && "삭제하고 싶은 링크가 어셈블리에 없습니다.");
			map< string, shared_ptr<Link> >::iterator link_iter = _link.find(link_pointer->getName());
			assert(link_iter->second == link_pointer && "삭제하고 싶은 링크의 이름을 갖는 링크는 존재하지만 실제로는 다른 링크입니다. 확인바랍니다.");

			for (std::list< Connection >::iterator iter = _connection.begin(); iter != _connection.end(); )
			{
				if (iter->_mountLink_pointer == link_pointer || iter->_actionLink_pointer == link_pointer)
				{
					_joint.erase(iter->_joint_pointer->getName());
					iter = _connection.erase(iter);
				}
				else iter++;
			}
			map< string, Math::SE3 >  marker_map = link_pointer->getMarkerMap();
			for (map< string, Math::SE3 >::iterator miter = marker_map.begin(); miter != marker_map.end(); miter++)
			{
				_marker.erase(miter->first);
			}
			_link.erase(link_pointer->getName());
		}

		void Assembly::addJoint(const std::shared_ptr<Joint>& joint_pointer, const std::string& mountLM_name, const std::string& actionLM_name, const Math::SE3& mountLM_T, const Math::SE3& actionLM_T)
		{
			utils::Log(_lock > 0, "이 어셈블리는 이미 System에 들어가있습니다. 구조를 수정할 수 없습니다.", true);

			assert(!findNameList(joint_pointer->getName()) && "주어진 조인트의 이름과 같은 이름을 갖는 컴포넌트가 이미 어셈블리에 있습니다.");
			assert((isLink(mountLM_name) || isMarker(mountLM_name)) && "mountLM_name은 링크 또는 마커의 이름이어야 합니다.");
			assert((isLink(actionLM_name) || isMarker(actionLM_name)) && "actionLM_name은 링크 또는 마커의 이름이어야 합니다.");

			_joint.insert(pair< string, shared_ptr<Joint> >(joint_pointer->getName(), joint_pointer));

			Math::SE3 mountT, actionT;
			map< string, shared_ptr<Link> >::iterator mount_iter = _link.find(mountLM_name);
			map< string, shared_ptr<Link> >::iterator action_iter = _link.find(actionLM_name);
			if (mount_iter == _link.end())
			{
				mount_iter = _marker.find(mountLM_name);
				mountT = mount_iter->second->getMarker(mountLM_name);
			}
			if (action_iter == _link.end())
			{
				action_iter = _marker.find(actionLM_name);
				actionT = action_iter->second->getMarker(actionLM_name);
			}

			_connection.push_back(Connection(joint_pointer, mount_iter->second, action_iter->second, mountT*mountLM_T, actionLM_T*actionT.inverse()));
		}

		void Assembly::deleteJoint(const std::shared_ptr<Joint>& joint_pointer)
		{
			utils::Log(_lock > 0, "이 어셈블리는 이미 System에 들어가있습니다. 구조를 수정할 수 없습니다.", true);

			assert(isJoint(joint_pointer->getName()) && "삭제하고 싶은 조인트가 어셈블리에 없습니다.");
			map< string, shared_ptr<Joint> >::iterator link_iter = _joint.find(joint_pointer->getName());
			assert(link_iter->second == joint_pointer && "삭제하고 싶은 조인트의 이름을 갖는 링크는 존재하지만 실제로는 다른 조인트입니다. 확인바랍니다.");

			for (std::list< Connection >::iterator iter = _connection.begin(); iter != _connection.end(); )
			{
				if (iter->_joint_pointer == joint_pointer)
				{
					iter = _connection.erase(iter);
					break;
				}
			}

			_joint.erase(joint_pointer->getName());
		}

		bool Assembly::changeName(const std::string& old_name, const std::string& new_name)
		{
			utils::Log(_lock > 0, "이 어셈블리는 이미 System에 들어가있습니다. 구조를 수정할 수 없습니다.", true);

			if (isLink(new_name) | isJoint(new_name) | isMarker(new_name))
			{
				return false;
			}

			if (isLink(old_name))
			{
				shared_ptr<Link> link = findLink(old_name);
				link->_name = new_name;
				_link.erase(old_name);
				_link.insert(pair< string, shared_ptr<Link> >(new_name, link));
			}
			else if (isJoint(old_name))
			{
				shared_ptr<Joint> joint = findJoint(old_name);
				joint->_name = new_name;
				_joint.erase(old_name);
				_joint.insert(pair< string, shared_ptr<Joint> >(new_name, joint));
			}
			else if (isMarker(old_name))
			{
				shared_ptr<Link> link = findMarker(old_name);
				Math::SE3 marker_T = link->getMarker(old_name);
				link->addMarker(new_name, marker_T);
				_marker.erase(old_name);
				_marker.insert(pair< string, shared_ptr<Link> >(new_name, link));
			}
			return false;
		}

		bool Assembly::changeNameAll(const std::string& prefix)
		{
			utils::Log(_lock > 0, "이 어셈블리는 이미 System에 들어가있습니다. 구조를 수정할 수 없습니다.", true);

			bool flag = true;

			list<string> linklist = getLinkNameList();
			list<string> jointlist = getJointNameList();
			list<string> markerlist = getMarkerNameList();

			for (list<string>::iterator iter = linklist.begin(); iter != linklist.end(); iter++)
			{
				flag &= changeName(*iter, prefix + (*iter));
			}
			for (list<string>::iterator iter = jointlist.begin(); iter != jointlist.end(); iter++)
			{
				flag &= changeName(*iter, prefix + (*iter));
			}
			for (list<string>::iterator iter = markerlist.begin(); iter != markerlist.end(); iter++)
			{
				flag &= changeName(*iter, prefix + (*iter));
			}

			return flag;
		}

		Assembly Assembly::copy(const std::string& prefix) const
		{
			Assembly _clone = *this;
			_clone.changeNameAll(prefix);
			return _clone;
		}
	}
}