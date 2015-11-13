/**
*	\file	Assembly.h
*	\date	2015.11.07
*	\author	Keunjun (ckj@robotics.snu.ac.kr)
*	\brief	Assembly�� ������ �����ϴ� Ŭ���� ����
*/

#pragma once

#include <string>
#include <map>
#include <memory>

#include "Link.h"
#include "Joint.h"

namespace rovin
{
	namespace Model
	{
		class Joint;
		class Link;

		/**
		*	\class Assembly
		*	\brief Link�� joint�� ��������ְ� �𵨿� ���� ������ �����ϴ� Ŭ�����Դϴ�.
		*/
		class Assembly
		{
		public:
			/**
			*	\class Mate
			*	\brief Mate ������ �����մϴ�.
			*/
			class Connection
			{
			public:
				Connection(const std::shared_ptr<Joint> joint_pointer,
					const std::shared_ptr<Link> mountLink_pointer,
					const std::shared_ptr<Link> actionLink_pointer,
					const Math::SE3 mountLink_T,
					const Math::SE3 actionLink_T
					) : _joint_pointer(joint_pointer), _mountLink_pointer(mountLink_pointer), _actionLink_pointer(actionLink_pointer),
					_mountLink_T(mountLink_T), _actionLink_T(actionLink_T) {}

				std::shared_ptr<Joint> _joint_pointer; ///< ����Ʈ ������
				std::shared_ptr<Link> _mountLink_pointer; ///< ����Ʈ�� ��ġ�Ǵ� ��ũ�� ������
				std::shared_ptr<Link> _actionLink_pointer; ///< ����Ʈ�� ����Ǿ� �����̴� ��ũ�� ������
				Math::SE3 _mountLink_T; ///< ����Ʈ�� ��ġ�Ǵ� ��ũ�� �����ӿ��� �ٶ� ����Ʈ�� ������
				Math::SE3 _actionLink_T; ///< ����Ʈ �����ӿ��� �ٶ� ����Ʈ�� ����Ǿ� �����̴� ��ũ�� ������
			};

			/// ������
			Assembly() : _lock(), _link(), _joint() {}
			/// ���� ������
			Assembly(const Assembly& operand);

			/// �Ҹ���
			~Assembly();

			/// ���� ������ ����Ʈ���� ���� �մϴ�. copy�ʹ� �ٸ��� link, joint ���� ��ü ���������� �ʽ��ϴ�.
			Assembly& operator = (const Assembly& operand ///< �����ϰ� ���� assembly
				);
			/// �ΰ��� assembly�� �ϳ��� ����ϴ�.
			Assembly operator + (const Assembly& operand ///< ���� assembly�� �Բ� �߰��ϰ� ���� assembly
				);
			/// ������ assembly�� �ٸ� ������� �߰��մϴ�.
			Assembly& operator += (const Assembly& operand ///< ���� assembly�� �߰��ϰ� ���� assembly
				);

			/// LOCK
			void LOCK();

			/// UNLOCK
			void UNLOCK();

			/// Link�� �߰��մϴ�.
			std::shared_ptr<Link>& addLink(const std::shared_ptr<Link>& link_pointer ///< �߰��ϰ� ���� ��ũ�� ������
				);

			/// Link�� �����մϴ�. Link�� �����ϴ� ��� �� link�� ����Ǿ��ִ� ��� ����Ʈ�� ���ŵ˴ϴ�.
			void deleteLink(const std::shared_ptr<Link>& link_pointer ///< �����ϰ� ���� ��ũ�� ������
				);
			/// Link�� �����մϴ�.
			void deleteLink(const std::string& link_name /// �����ϰ� ���� ��ũ�� �̸�
				)
			{
				deleteLink(findLink(link_name));
			}

			/// Joint�� �߰��մϴ�.
			void addJoint(const std::shared_ptr<Joint>& joint_pointer, ///< �߰��ϰ� ���� ����Ʈ�� ������
				const std::shared_ptr<Link>& mountLink_pointer, ///< Joint�� ��ġ�ϰ� �� ��ũ�� ������
				const std::shared_ptr<Link>& actionLink_pointer, ///< Joint�� ����Ǿ� �����̰� �Ǵ� ��ũ�� ������
				const Math::SE3& mountLink_T = (Math::SE3()), ///< Mount ��ũ �����ӿ��� �ٶ� joint ������
				const Math::SE3& actionLink_T = (Math::SE3()) ///< Joint �����ӿ��� �ٶ� action ��ũ ������
				)
			{
				addJoint(joint_pointer, addLink(mountLink_pointer), addLink(actionLink_pointer), mountLink_T, actionLink_T);
			}
			/// Joint�� �߰��մϴ�.
			void addJoint(const std::shared_ptr<Joint>& joint_pointer, ///< �߰��ϰ� ���� ����Ʈ�� ������
				const std::string& mountLM_name, ///< Joint�� ��ġ�ϰ� �� ��ũ �Ǵ� ��Ŀ�� �̸�
				const std::shared_ptr<Link>& actionLink_pointer, ///< Joint�� ����Ǿ� �����̰� �Ǵ� ��ũ�� ������
				const Math::SE3& mountLM_T = (Math::SE3()), ///< Mount ��ũ �Ǵ� ��Ŀ �����ӿ��� �ٶ� joint ������
				const Math::SE3& actionLink_T = (Math::SE3()) ///< Joint �����ӿ��� �ٶ� action ��ũ ������
				)
			{
				addJoint(joint_pointer, mountLM_name, addLink(actionLink_pointer), mountLM_T, actionLink_T);
			}
			/// Joint�� �߰��մϴ�.
			void addJoint(const std::shared_ptr<Joint>& joint_pointer, ///< �߰��ϰ� ���� ����Ʈ�� ������
				const std::shared_ptr<Link>& mountLink_pointer, ///< Joint�� ��ġ�ϰ� �� ��ũ�� ������
				const std::string& actionLM_name, ///< Joint�� ����Ǿ� �����̰� �Ǵ� ��ũ �Ǵ� ��Ŀ�� �̸�
				const Math::SE3& mountLink_T = (Math::SE3()), ///< Mount ��ũ �����ӿ��� �ٶ� joint ������
				const Math::SE3& actionLM_T = (Math::SE3()) ///< Joint �����ӿ��� �ٶ� action ��ũ �Ǵ� ��Ŀ ������
				)
			{
				addJoint(joint_pointer, addLink(mountLink_pointer), actionLM_name, mountLink_T, actionLM_T);
			}
			/// Joint�� �߰��մϴ�.
			void addJoint(const std::shared_ptr<Joint>& joint_pointer, ///< �߰��ϰ� ���� ����Ʈ�� ������
				const std::string& mountLM_name, ///< Joint�� ��ġ�ϰ� �� ��ũ �Ǵ� ��Ŀ�� �̸�
				const std::string& actionLM_name, ///< Joint�� ����Ǿ� �����̰� �Ǵ� ��ũ �Ǵ� ��Ŀ�� �̸�
				const Math::SE3& mountLM_T = (Math::SE3()), ///< Mount ��ũ �Ǵ� ��Ŀ �����ӿ��� �ٶ� joint ������
				const Math::SE3& actionLM_T = (Math::SE3()) ///< Joint �����ӿ��� �ٶ� action ��ũ �Ǵ� ��Ŀ ������
				);

			/// Joint�� �����մϴ�.
			void deleteJoint(const std::shared_ptr<Joint>& joint_pointer ///< �����ϰ� ���� ����Ʈ�� ������
				);
			/// Joint�� �����մϴ�.
			void deleteJoint(const std::string& joint_name ///< �����ϰ� ���� ����Ʈ�� �̸�
				)
			{
				deleteJoint(findJoint(joint_name));
			}
			/// Joint�� ��� �����մϴ�.
			void clearJoint()
			{
				std::map< std::string, std::shared_ptr<Joint> >::iterator iter;
				while ((iter = _joint.begin()) != _joint.end())
				{
					deleteJoint(iter->second);
				}
			}

			/// Link�� �̸��� �̿��ؼ� ã���ϴ�.
			std::shared_ptr<Link>& findLink(const std::string& link_name)
			{
				assert(isLink(link_name) && "ã���� �ϴ� �̸��� ���� ��ũ�� ������� �������� �ʽ��ϴ�..");
				return _link.find(link_name)->second;
			}
			/// Joint�� �̸��� �̿��ؼ� ã���ϴ�.
			std::shared_ptr<Joint>& findJoint(const std::string& joint_name)
			{
				assert(isJoint(joint_name) && "ã���� �ϴ� �̸��� ���� ����Ʈ�� ������� �������� �ʽ��ϴ�..");
				return _joint.find(joint_name)->second;
			}
			/// Marker�� �̸��� �̿��ؼ� ����Ǿ��ִ� Link�� ã���ϴ�.
			std::shared_ptr<Link>& findMarker(const std::string& marker_name)
			{
				assert(isLink(marker_name) && "ã���� �ϴ� �̸��� ���� ��Ŀ�� ������� �������� �ʽ��ϴ�..");
				return _link.find(marker_name)->second;
			}
			/// Link�� �̸��� �̿��ؼ� ã���ϴ�.
			const std::shared_ptr<Link>& findLink(const std::string& link_name) const
			{
				assert(isLink(link_name) && "ã���� �ϴ� �̸��� ���� ��ũ�� ������� �������� �ʽ��ϴ�..");
				return _link.find(link_name)->second;
			}
			/// Joint�� �̸��� �̿��ؼ� ã���ϴ�.
			const std::shared_ptr<Joint>& findJoint(const std::string& joint_name) const
			{
				assert(isJoint(joint_name) && "ã���� �ϴ� �̸��� ���� ����Ʈ�� ������� �������� �ʽ��ϴ�..");
				return _joint.find(joint_name)->second;
			}
			/// Marker�� �̸��� �̿��ؼ� ����Ǿ��ִ� Link�� ã���ϴ�.
			const std::shared_ptr<Link>& findMarker(const std::string& marker_name) const
			{
				assert(isLink(marker_name) && "ã���� �ϴ� �̸��� ���� ��Ŀ�� ������� �������� �ʽ��ϴ�..");
				return _link.find(marker_name)->second;
			}

			/// Link�� �̸��� �̿��ؼ� ã���ϴ�.
			bool isLink(const std::string& link_name) const
			{
				return _link.find(link_name) != _link.end();
			}
			/// Joint�� �̸��� �̿��ؼ� ã���ϴ�.
			bool isJoint(const std::string& joint_name) const
			{
				return _joint.find(joint_name) != _joint.end();
			}
			/// Marker�� �̸��� �̿��ؼ� ����Ǿ��ִ� Link�� ã���ϴ�.
			bool isMarker(const std::string& marker_name) const
			{
				return _marker.find(marker_name) != _marker.end();
			}

			/// Link���� �̸� ����� ������ �ɴϴ�.
			std::list<std::string> getLinkNameList() const
			{
				std::list<std::string> result;
				for (std::map< std::string, std::shared_ptr<Link> >::const_iterator iter = _link.begin(); iter != _link.end(); iter++)
				{
					result.push_back(iter->first);
				}
				return result;
			}
			/// Joint���� �̸� ����� ������ �ɴϴ�.
			std::list<std::string> getJointNameList() const
			{
				std::list<std::string> result;
				for (std::map< std::string, std::shared_ptr<Joint> >::const_iterator iter = _joint.begin(); iter != _joint.end(); iter++)
				{
					result.push_back(iter->first);
				}
				return result;
			}
			/// Marker���� �̸� ����� ������ �ɴϴ�.
			std::list<std::string> getMarkerNameList() const
			{
				std::list<std::string> result;
				for (std::map< std::string, std::shared_ptr<Link> >::const_iterator iter = _marker.begin(); iter != _marker.end(); iter++)
				{
					result.push_back(iter->first);
				}
				return result;
			}

			/// �̸� ����Ʈ���� ���� �̸��� �ִ��� �ִ��� Ȯ���մϴ�.
			bool findNameList(const std::string& name) const
			{
				return isLink(name) | isJoint(name) | isMarker(name);
			}

			/// ���� ���縦 �մϴ�.
			Assembly copy(const std::string& prefix ///< ��ũ, ����Ʈ, ��Ŀ�� �̸� �տ� ���� ���λ�
				) const;

			/// ���� �������� ������ �ɴϴ�.
			const std::list< Connection >& getConnection() const
			{
				return _connection;
			}

		private:
			/**
			*	\return �̸��� ���� �� ��� true, �̸��� ������� ���� ��� false
			*	\brief �̸� ����
			*/
			bool changeName(const std::string& old_name, ///< ���� ���� �Ǿ��ִ� �̸�
				const std::string& new_name ///< �ٲٰ� ���� �̸�
				);
			/**
			*	\return �̸��� ���� �� ��� true, �̸��� ������� ���� ��� false
			*	\brief ��� ��ũ, ����Ʈ, ��Ŀ �̸� ����
			*/
			bool changeNameAll(const std::string& prefix ///< ��ũ, ����Ʈ, ��Ŀ�� �̸� �տ� ���� ���λ�
				);

		private:
			unsigned int _lock;

			std::map< std::string, std::shared_ptr<Link> > _link;
			std::map< std::string, std::shared_ptr<Joint> > _joint;
			std::map< std::string, std::shared_ptr<Link> > _marker;

			std::list< Connection > _connection;
		};
	}
}