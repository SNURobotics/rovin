/**
*	\file	Assembly.h
*	\date	2015.11.07
*	\author	Keunjun (ckj@robotics.snu.ac.kr)
*	\brief	Assembly의 정보를 저장하는 클래스 정의
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
		*	\brief Link와 joint를 연결시켜주고 모델에 대한 정보를 저장하는 클래스입니다.
		*/
		class Assembly
		{
		public:
			/**
			*	\class Mate
			*	\brief Mate 정보를 저장합니다.
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

				std::shared_ptr<Joint> _joint_pointer; ///< 조인트 포인터
				std::shared_ptr<Link> _mountLink_pointer; ///< 조인트가 설치되는 링크의 포인터
				std::shared_ptr<Link> _actionLink_pointer; ///< 조인트에 연결되어 움직이는 링크의 포인터
				Math::SE3 _mountLink_T; ///< 조인트가 설치되는 링크의 프레임에서 바라본 조인트의 프레임
				Math::SE3 _actionLink_T; ///< 조인트 프레임에서 바라본 조인트에 연결되어 움직이는 링크의 프레임
			};

			/// 생성자
			Assembly() : _lock(), _link(), _joint() {}
			/// 복사 생성자
			Assembly(const Assembly& operand);

			/// 소멸자
			~Assembly();

			/// 대입 연산자 리스트들을 복사 합니다. copy와는 다르게 link, joint 등을 객체 복사하지는 않습니다.
			Assembly& operator = (const Assembly& operand ///< 복사하고 싶은 assembly
				);
			/// 두개의 assembly를 하나로 만듭니다.
			Assembly operator + (const Assembly& operand ///< 현재 assembly와 함께 추가하고 싶은 assembly
				);
			/// 현재의 assembly에 다른 어셈블리를 추가합니다.
			Assembly& operator += (const Assembly& operand ///< 현재 assembly에 추가하고 싶은 assembly
				);

			/// LOCK
			void LOCK();

			/// UNLOCK
			void UNLOCK();

			/// Link를 추가합니다.
			std::shared_ptr<Link>& addLink(const std::shared_ptr<Link>& link_pointer ///< 추가하고 싶은 링크의 포인터
				);

			/// Link를 제거합니다. Link를 제거하는 경우 그 link와 연결되어있는 모든 조인트도 제거됩니다.
			void deleteLink(const std::shared_ptr<Link>& link_pointer ///< 제거하고 싶은 링크의 포인터
				);
			/// Link를 제거합니다.
			void deleteLink(const std::string& link_name /// 제거하고 싶은 링크의 이름
				)
			{
				deleteLink(findLink(link_name));
			}

			/// Joint를 추가합니다.
			void addJoint(const std::shared_ptr<Joint>& joint_pointer, ///< 추가하고 싶은 조인트의 포인터
				const std::shared_ptr<Link>& mountLink_pointer, ///< Joint를 설치하게 될 링크의 포인터
				const std::shared_ptr<Link>& actionLink_pointer, ///< Joint에 연결되어 움직이게 되는 링크의 포인터
				const Math::SE3& mountLink_T = (Math::SE3()), ///< Mount 링크 프레임에서 바라본 joint 프레임
				const Math::SE3& actionLink_T = (Math::SE3()) ///< Joint 프레임에서 바라본 action 링크 프레임
				)
			{
				addJoint(joint_pointer, addLink(mountLink_pointer), addLink(actionLink_pointer), mountLink_T, actionLink_T);
			}
			/// Joint를 추가합니다.
			void addJoint(const std::shared_ptr<Joint>& joint_pointer, ///< 추가하고 싶은 조인트의 포인터
				const std::string& mountLM_name, ///< Joint를 설치하게 될 링크 또는 마커의 이름
				const std::shared_ptr<Link>& actionLink_pointer, ///< Joint에 연결되어 움직이게 되는 링크의 포인터
				const Math::SE3& mountLM_T = (Math::SE3()), ///< Mount 링크 또는 마커 프레임에서 바라본 joint 프레임
				const Math::SE3& actionLink_T = (Math::SE3()) ///< Joint 프레임에서 바라본 action 링크 프레임
				)
			{
				addJoint(joint_pointer, mountLM_name, addLink(actionLink_pointer), mountLM_T, actionLink_T);
			}
			/// Joint를 추가합니다.
			void addJoint(const std::shared_ptr<Joint>& joint_pointer, ///< 추가하고 싶은 조인트의 포인터
				const std::shared_ptr<Link>& mountLink_pointer, ///< Joint를 설치하게 될 링크의 포인터
				const std::string& actionLM_name, ///< Joint에 연결되어 움직이게 되는 링크 또는 마커의 이름
				const Math::SE3& mountLink_T = (Math::SE3()), ///< Mount 링크 프레임에서 바라본 joint 프레임
				const Math::SE3& actionLM_T = (Math::SE3()) ///< Joint 프레임에서 바라본 action 링크 또는 마커 프레임
				)
			{
				addJoint(joint_pointer, addLink(mountLink_pointer), actionLM_name, mountLink_T, actionLM_T);
			}
			/// Joint를 추가합니다.
			void addJoint(const std::shared_ptr<Joint>& joint_pointer, ///< 추가하고 싶은 조인트의 포인터
				const std::string& mountLM_name, ///< Joint를 설치하게 될 링크 또는 마커의 이름
				const std::string& actionLM_name, ///< Joint에 연결되어 움직이게 되는 링크 또는 마커의 이름
				const Math::SE3& mountLM_T = (Math::SE3()), ///< Mount 링크 또는 마커 프레임에서 바라본 joint 프레임
				const Math::SE3& actionLM_T = (Math::SE3()) ///< Joint 프레임에서 바라본 action 링크 또는 마커 프레임
				);

			/// Joint를 제거합니다.
			void deleteJoint(const std::shared_ptr<Joint>& joint_pointer ///< 제거하고 싶은 조인트의 포인터
				);
			/// Joint를 제거합니다.
			void deleteJoint(const std::string& joint_name ///< 제거하고 싶은 조인트의 이름
				)
			{
				deleteJoint(findJoint(joint_name));
			}
			/// Joint를 모두 제거합니다.
			void clearJoint()
			{
				std::map< std::string, std::shared_ptr<Joint> >::iterator iter;
				while ((iter = _joint.begin()) != _joint.end())
				{
					deleteJoint(iter->second);
				}
			}

			/// Link를 이름을 이용해서 찾습니다.
			std::shared_ptr<Link>& findLink(const std::string& link_name)
			{
				assert(isLink(link_name) && "찾고자 하는 이름을 갖는 링크는 어셈블리에 존재하지 않습니다..");
				return _link.find(link_name)->second;
			}
			/// Joint를 이름을 이용해서 찾습니다.
			std::shared_ptr<Joint>& findJoint(const std::string& joint_name)
			{
				assert(isJoint(joint_name) && "찾고자 하는 이름을 갖는 조인트는 어셈블리에 존재하지 않습니다..");
				return _joint.find(joint_name)->second;
			}
			/// Marker를 이름을 이용해서 연결되어있는 Link를 찾습니다.
			std::shared_ptr<Link>& findMarker(const std::string& marker_name)
			{
				assert(isLink(marker_name) && "찾고자 하는 이름을 갖는 마커는 어셈블리에 존재하지 않습니다..");
				return _link.find(marker_name)->second;
			}
			/// Link를 이름을 이용해서 찾습니다.
			const std::shared_ptr<Link>& findLink(const std::string& link_name) const
			{
				assert(isLink(link_name) && "찾고자 하는 이름을 갖는 링크는 어셈블리에 존재하지 않습니다..");
				return _link.find(link_name)->second;
			}
			/// Joint를 이름을 이용해서 찾습니다.
			const std::shared_ptr<Joint>& findJoint(const std::string& joint_name) const
			{
				assert(isJoint(joint_name) && "찾고자 하는 이름을 갖는 조인트는 어셈블리에 존재하지 않습니다..");
				return _joint.find(joint_name)->second;
			}
			/// Marker를 이름을 이용해서 연결되어있는 Link를 찾습니다.
			const std::shared_ptr<Link>& findMarker(const std::string& marker_name) const
			{
				assert(isLink(marker_name) && "찾고자 하는 이름을 갖는 마커는 어셈블리에 존재하지 않습니다..");
				return _link.find(marker_name)->second;
			}

			/// Link를 이름을 이용해서 찾습니다.
			bool isLink(const std::string& link_name) const
			{
				return _link.find(link_name) != _link.end();
			}
			/// Joint를 이름을 이용해서 찾습니다.
			bool isJoint(const std::string& joint_name) const
			{
				return _joint.find(joint_name) != _joint.end();
			}
			/// Marker를 이름을 이용해서 연결되어있는 Link를 찾습니다.
			bool isMarker(const std::string& marker_name) const
			{
				return _marker.find(marker_name) != _marker.end();
			}

			/// Link들의 이름 목록을 가지고 옵니다.
			std::list<std::string> getLinkNameList() const
			{
				std::list<std::string> result;
				for (std::map< std::string, std::shared_ptr<Link> >::const_iterator iter = _link.begin(); iter != _link.end(); iter++)
				{
					result.push_back(iter->first);
				}
				return result;
			}
			/// Joint들의 이름 목록을 가지고 옵니다.
			std::list<std::string> getJointNameList() const
			{
				std::list<std::string> result;
				for (std::map< std::string, std::shared_ptr<Joint> >::const_iterator iter = _joint.begin(); iter != _joint.end(); iter++)
				{
					result.push_back(iter->first);
				}
				return result;
			}
			/// Marker들의 이름 목록을 가지고 옵니다.
			std::list<std::string> getMarkerNameList() const
			{
				std::list<std::string> result;
				for (std::map< std::string, std::shared_ptr<Link> >::const_iterator iter = _marker.begin(); iter != _marker.end(); iter++)
				{
					result.push_back(iter->first);
				}
				return result;
			}

			/// 이름 리스트에서 현재 이름이 있는지 있는지 확인합니다.
			bool findNameList(const std::string& name) const
			{
				return isLink(name) | isJoint(name) | isMarker(name);
			}

			/// 깊은 복사를 합니다.
			Assembly copy(const std::string& prefix ///< 링크, 조인트, 마커의 이름 앞에 붙일 접두사
				) const;

			/// 연결 정보들을 가지고 옵니다.
			const std::list< Connection >& getConnection() const
			{
				return _connection;
			}

		private:
			/**
			*	\return 이름이 변경 된 경우 true, 이름이 변경되지 않은 경우 false
			*	\brief 이름 변경
			*/
			bool changeName(const std::string& old_name, ///< 현재 설정 되어있는 이름
				const std::string& new_name ///< 바꾸고 싶은 이름
				);
			/**
			*	\return 이름이 변경 된 경우 true, 이름이 변경되지 않은 경우 false
			*	\brief 모든 링크, 조인트, 마커 이름 변경
			*/
			bool changeNameAll(const std::string& prefix ///< 링크, 조인트, 마커의 이름 앞에 붙일 접두사
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