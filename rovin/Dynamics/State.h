/**
*	\file	State.h
*	\date	2015.11.09
*	\author	Keunjun (ckj@robotics.snu.ac.kr)
*	\brief	State�� ������ �����ϴ� Ŭ���� ����
*/

#pragma once

#include <map>

#include <rovin/utils/Diagnostic.h>

#include <rovin/Math/Constant.h>
#include <rovin/Math/LieGroup.h>

#include <rovin/Model/Assembly.h>

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

					internalF = Math::dse3();
				}

				Math::SE3 T;
				Math::se3 V;

				Math::dse3 internalF;
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
				}

				unsigned int dof;

				Math::VectorX q;
				Math::VectorX qdot;
				
				Math::VectorX tau;

				Math::dse3 constraintF;
			};

			/// ������
			State(const Model::Assembly& model, const std::list< std::string >& activeJointList);

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
				std::map< std::string, LinkState >::iterator iter = _linkState.find(link_name);
				utils::Log(iter == _linkState.end(), "ã���� �ϴ� �̸��� ���� ��ũ�� �������� �ʽ��ϴ�.", true);
				return iter->second;
			}
			/// Joint�� ���¸� ������ �ɴϴ�.
			JointState& getJointState(const std::string& joint_name)
			{
				std::map< std::string, JointState >::iterator iter = _jointState.find(joint_name);
				utils::Log(iter == _jointState.end(), "ã���� �ϴ� �̸��� ���� ����Ʈ�� �������� �ʽ��ϴ�.", true);
				return iter->second;
			}

			/// Active joint�� ����Ʈ�� ������ �ɴϴ�.
			const std::list< std::string >& getActiveJointList()
			{
				return _activeJointList;
			}

			/// ����Ʈ �Է�(VectorX)�� (list)�� ��ȯ�մϴ�. ex) 1, 2, 3, 4, 5 -> 1, 2 / 3 / 4, 5 (2dof, 1dof, 2dof)
			std::list< Math::VectorX > vector2list(const Math::VectorX& _q);
			/// ����Ʈ �Է�(list)�� VectorX�� ��ȯ�մϴ�. ex) 1, 2 / 3 / 4, 5 (2dof, 1dof, 2dof) -> 1, 2, 3, 4, 5
			Math::VectorX list2vector(const std::list< Math::VectorX >& _q);

			/// Joint�� q�� �����մϴ�.
			void setJoint_q(const Math::VectorX& _q);
			/// Joint�� q�� �����մϴ�.
			void setJoint_qdot(const Math::VectorX& _qdot);
			/// Joint�� tau�� �����մϴ�.
			void setJoint_tau(const Math::VectorX& _tau);

		private:
			unsigned int _dof;
			unsigned int _total_dof;

			std::map< std::string, LinkState > _linkState;
			std::map< std::string, JointState > _jointState;

			std::list< std::string > _activeJointList;
		};
	}
}