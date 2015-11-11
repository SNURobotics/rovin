/**
*	\file	State.h
*	\date	2015.11.09
*	\author	Keunjun (ckj@robotics.snu.ac.kr)
*	\brief	State의 정보를 저장하는 클래스 정의
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
		*	\brief State를 생성하고 처리하는 클래스
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

			/// 생성자
			State(const Model::Assembly& model, const std::list< std::string >& activeJointList);

			/**
			*	\return 추가에 성공하면 true, 추가에 실패하면 false
			*	\brief Active joint 추가합니다.
			*/
			bool addActiveJoint(const std::string& activeJoint);
			/**
			*	\return 제거에 성공하면 true, 제거에 실패하면 false
			*	\brief Active joint 제거합니다.
			*/
			bool eraseActiveJoint(const std::string& activeJoint);

			/// Link의 상태를 가지고 옵니다.
			LinkState& getLinkState(const std::string& link_name)
			{
				std::map< std::string, LinkState >::iterator iter = _linkState.find(link_name);
				utils::Log(iter == _linkState.end(), "찾고자 하는 이름을 갖는 링크가 존재하지 않습니다.", true);
				return iter->second;
			}
			/// Joint의 상태를 가지고 옵니다.
			JointState& getJointState(const std::string& joint_name)
			{
				std::map< std::string, JointState >::iterator iter = _jointState.find(joint_name);
				utils::Log(iter == _jointState.end(), "찾고자 하는 이름을 갖는 조인트가 존재하지 않습니다.", true);
				return iter->second;
			}

			/// Active joint의 리스트를 가지고 옵니다.
			const std::list< std::string >& getActiveJointList()
			{
				return _activeJointList;
			}

			/// 조인트 입력(VectorX)를 (list)로 변환합니다. ex) 1, 2, 3, 4, 5 -> 1, 2 / 3 / 4, 5 (2dof, 1dof, 2dof)
			std::list< Math::VectorX > vector2list(const Math::VectorX& _q);
			/// 조인트 입력(list)를 VectorX로 변환합니다. ex) 1, 2 / 3 / 4, 5 (2dof, 1dof, 2dof) -> 1, 2, 3, 4, 5
			Math::VectorX list2vector(const std::list< Math::VectorX >& _q);

			/// Joint에 q를 설정합니다.
			void setJoint_q(const Math::VectorX& _q);
			/// Joint에 q를 설정합니다.
			void setJoint_qdot(const Math::VectorX& _qdot);
			/// Joint에 tau를 설정합니다.
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