/**
*	\file	State.h
*	\date	2015.11.09
*	\author	Keunjun (ckj@robotics.snu.ac.kr)
*	\brief	State�� ������ �����ϴ� Ŭ���� ����
*/

#pragma once

#include <map>
#include <vector>

#include <rovin/utils/Diagnostic.h>

#include <rovin/Math/Constant.h>
#include <rovin/Math/LieGroup.h>

#include <rovin/Model/Assembly.h>

#include "System.h"

namespace rovin
{
	namespace Dynamics
	{
		/**
		*	\class State
		*	\brief State�� �����ϰ� ó���ϴ� Ŭ����
		*/
		class State
		{
		public:
			class LinkState
			{
			public:
				LinkState()
				{
					T = Math::SE3();
					V = Math::se3();
				}

				Math::SE3 T;
				Math::se3 V;
			};

			class JointState
			{
			public:
				JointState(const unsigned int _dof)
				{
					dof = _dof;

					q = Math::VectorX(_dof);
					qdot = Math::VectorX(_dof);

					tau = Math::VectorX(_dof);

					constraintF = Math::dse3();

					index = 0;
				}

				unsigned int dof;

				Math::VectorX q;
				Math::VectorX qdot;
				
				Math::VectorX tau;

				Math::dse3 constraintF;

				unsigned int index;
			};

			/// ������
			State(const System& system, const std::list< std::string >& activeJointList);

			/// �� �������� ������ �ɴϴ�.
			unsigned int getTotalDof() const
			{
				return _total_dof;
			}
			/// Active joint �������� ������ �ɴϴ�.
			unsigned int getActiveJointDof() const
			{
				return _activejoint_dof;
			}

			/**
			*	\return �߰��� �����ϸ� true, �߰��� �����ϸ� false
			*	\brief Active joint �߰��մϴ�.
			*/
			bool addActiveJoint(const std::string& activeJoint);
			/**
			*	\return ���ſ� �����ϸ� true, ���ſ� �����ϸ� false
			*	\brief Active joint �����մϴ�.
			*/
			bool eraseActiveJoint(const std::string& activeJoint);

			/// Link�� ���¸� ������ �ɴϴ�.
			LinkState& getLinkState(const std::string& link_name)
			{
				std::map< std::string, unsigned int >::iterator iter = _linkMap.find(link_name);
				utils::Log(iter == _linkMap.end(), "ã���� �ϴ� �̸��� ���� ��ũ�� �������� �ʽ��ϴ�.", true);
				return _linkState[iter->second];
			}
			/// Link�� ���¸� ������ �ɴϴ�.
			LinkState& getLinkState(const unsigned int& link_num)
			{
				return _linkState[link_num];
			}
			/// Joint�� ���¸� ������ �ɴϴ�.
			JointState& getJointState(const std::string& joint_name)
			{
				std::map< std::string, unsigned int >::iterator iter = _jointMap.find(joint_name);
				utils::Log(iter == _jointMap.end(), "ã���� �ϴ� �̸��� ���� ����Ʈ�� �������� �ʽ��ϴ�.", true);
				return _jointState[iter->second];
			}
			/// Link�� ���¸� ������ �ɴϴ�.
			JointState& getJointState(const unsigned int& joint_num)
			{
				return _jointState[joint_num];
			}
			/// Link�� ���¸� ������ �ɴϴ�.
			const LinkState& getLinkState(const std::string& link_name) const
			{
				std::map< std::string, unsigned int >::const_iterator iter = _linkMap.find(link_name);
				utils::Log(iter == _linkMap.end(), "ã���� �ϴ� �̸��� ���� ��ũ�� �������� �ʽ��ϴ�.", true);
				return _linkState[iter->second];
			}
			/// Link�� ���¸� ������ �ɴϴ�.
			const LinkState& getLinkState(const unsigned int& link_num) const
			{
				return _linkState[link_num];
			}
			/// Joint�� ���¸� ������ �ɴϴ�.
			const JointState& getJointState(const std::string& joint_name) const
			{
				std::map< std::string, unsigned int >::const_iterator iter = _jointMap.find(joint_name);
				utils::Log(iter == _jointMap.end(), "ã���� �ϴ� �̸��� ���� ����Ʈ�� �������� �ʽ��ϴ�.", true);
				return _jointState[iter->second];
			}
			/// Link�� ���¸� ������ �ɴϴ�.
			const JointState& getJointState(const unsigned int& joint_num) const
			{
				return _jointState[joint_num];
			}

			/// Active joint���� Ȯ���մϴ�.
			bool isActiveJoint(const std::string& joint_name)
			{
				std::map< std::string, unsigned int >::iterator iter = _jointMap.find(joint_name);
				utils::Log(iter == _jointMap.end(), "ã���� �ϴ� �̸��� ���� ����Ʈ�� �������� �ʽ��ϴ�.", true);
				return _activeJoint[iter->second];
			}
			/// Active joint���� Ȯ���մϴ�.
			bool isActiveJoint(const unsigned int& joint_num) const
			{
				return _activeJoint[joint_num];
			}

			/// Active joint�� ����Ʈ�� ������ �ɴϴ�.
			const std::list< std::pair< std::string, unsigned int >>& getActiveJointList()
			{
				return _activeJointList;
			}
			/// Passive joint�� ����Ʈ�� ������ �ɴϴ�.
			const std::list< std::pair< std::string, unsigned int >>& getPassiveJointList()
			{
				return _passiveJointList;
			}

			/// ���� ��� �Ǵ� ������ ũ�⸦ ���ϱ� ���� �Լ�
			unsigned int getReturnDof(const System::RETURN_STATE& return_state);
			/// ���� ����� ������ݴϴ�.
			void makeReturnMatrix(Math::MatrixX& target, const Math::MatrixX& value, const unsigned int& row, const unsigned int& joint_num, const System::RETURN_STATE& return_state);

			/// Active joint�� q���� �����մϴ�.
			void setActiveJoint_q(const Math::VectorX& q);
			/// Passive joint�� q���� �����մϴ�.
			void setPassiveJoint_q(const Math::VectorX& q);
			/// Passive joint�� q���� ���մϴ�.
			void addPassiveJoint_q(const Math::VectorX& q);

		private:
			unsigned int _activejoint_dof;
			unsigned int _total_dof;

			std::vector< LinkState > _linkState;
			std::vector< JointState > _jointState;

			std::map< std::string, unsigned int > _linkMap;
			std::map< std::string, unsigned int > _jointMap;

			std::vector< unsigned int > _jointSystemIndex;

			std::vector< bool > _activeJoint;
			std::list< std::pair< std::string, unsigned int >> _activeJointList;
			std::list< std::pair< std::string, unsigned int >> _passiveJointList;
		};
	}
}