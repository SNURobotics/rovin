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
			/// 생성자
			Assembly() : _link(), _joint() {}

			/// 대입 연산자는 깊은 복사를 뜻합니다.
			Assembly& operator = (const Assembly& operand ///< 복사하고 싶은 assembly
				);
			/// 두개의 assembly를 하나로 만듭니다.
			Assembly operator + (const Assembly& operand ///< 현재 assembly와 함께 추가하고 싶은 assembly
				);
			/// 현재의 assembly에 다른 어셈블리를 추가합니다.
			Assembly& operator += (const Assembly& operand ///< 현재 assembly에 추가하고 싶은 assembly
				);

			/// Link를 추가합니다.
			void addLink(const Link* link_pointer ///< 추가하고 싶은 링크의 포인터
				);
			/// Link를 추가합니다.
			void addLink(const std::shared_ptr<Link>& link_pointer ///< 추가하고 싶은 링크의 포인터
				);

			/// Joint를 추가합니다.
			void addJoint(const std::shared_ptr<Joint>& joint_pointer, ///< 추가하고 싶은 조인트의 포인터
				const std::shared_ptr<Link>& mountLink, ///< Joint를 설치하게 될 링크의 포인터
				const std::shared_ptr<Link>& actionLink, ///< Joint에 연결되어 움직이게 되는 링크의 포인터
				const Math::SE3& mountLink_T = (Math::SE3()), ///< Joint 프레임에서 바라본 mount link 프레임
				const Math::SE3& actionLink_T = (Math::SE3()) ///< Joint 프레임에서 바라본 action link 프레임
				);

			/// Link를 이름을 이용해서 찾습니다.
			std::shared_ptr<Link>& findLink(const std::string& link_name);
			/// Joint를 이름을 이용해서 찾습니다.
			std::shared_ptr<Joint>& findJoint(const std::string& joint_name);

			/// Marker를 이름을 이용해서 연결되어있는 Link를 찾습니다.
			std::shared_ptr<Link>& findMarker(const std::string& marker_name);

			/// 깊은 복사
			std::shared_ptr<Assembly> copy();

		private:
			std::map< std::string, std::shared_ptr<Link> > _link;
			std::map< std::string, std::shared_ptr<Joint> > _joint;
		};
	}
}