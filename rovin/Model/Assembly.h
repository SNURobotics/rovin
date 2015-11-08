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
			/// ������
			Assembly() : _link(), _joint() {}

			/// ���� �����ڴ� ���� ���縦 ���մϴ�.
			Assembly& operator = (const Assembly& operand ///< �����ϰ� ���� assembly
				);
			/// �ΰ��� assembly�� �ϳ��� ����ϴ�.
			Assembly operator + (const Assembly& operand ///< ���� assembly�� �Բ� �߰��ϰ� ���� assembly
				);
			/// ������ assembly�� �ٸ� ������� �߰��մϴ�.
			Assembly& operator += (const Assembly& operand ///< ���� assembly�� �߰��ϰ� ���� assembly
				);

			/// Link�� �߰��մϴ�.
			void addLink(const Link* link_pointer ///< �߰��ϰ� ���� ��ũ�� ������
				);
			/// Link�� �߰��մϴ�.
			void addLink(const std::shared_ptr<Link>& link_pointer ///< �߰��ϰ� ���� ��ũ�� ������
				);

			/// Joint�� �߰��մϴ�.
			void addJoint(const std::shared_ptr<Joint>& joint_pointer, ///< �߰��ϰ� ���� ����Ʈ�� ������
				const std::shared_ptr<Link>& mountLink, ///< Joint�� ��ġ�ϰ� �� ��ũ�� ������
				const std::shared_ptr<Link>& actionLink, ///< Joint�� ����Ǿ� �����̰� �Ǵ� ��ũ�� ������
				const Math::SE3& mountLink_T = (Math::SE3()), ///< Joint �����ӿ��� �ٶ� mount link ������
				const Math::SE3& actionLink_T = (Math::SE3()) ///< Joint �����ӿ��� �ٶ� action link ������
				);

			/// Link�� �̸��� �̿��ؼ� ã���ϴ�.
			std::shared_ptr<Link>& findLink(const std::string& link_name);
			/// Joint�� �̸��� �̿��ؼ� ã���ϴ�.
			std::shared_ptr<Joint>& findJoint(const std::string& joint_name);

			/// Marker�� �̸��� �̿��ؼ� ����Ǿ��ִ� Link�� ã���ϴ�.
			std::shared_ptr<Link>& findMarker(const std::string& marker_name);

			/// ���� ����
			std::shared_ptr<Assembly> copy();

		private:
			std::map< std::string, std::shared_ptr<Link> > _link;
			std::map< std::string, std::shared_ptr<Joint> > _joint;
		};
	}
}