/**
*	\file	State.h
*	\date	2015.11.09
*	\author	Keunjun (ckj@robotics.snu.ac.kr)
*	\brief	State의 정보를 저장하는 클래스 정의
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

			/// 생성자
			State(const System& system, const std::list< std::string >& activeJointList);

			/// 총 자유도를 가지고 옵니다.
			unsigned int getTotalDof() const
			{
				return _total_dof;
			}
			/// Active joint 자유도를 가지고 옵니다.
			unsigned int getActiveJointDof() const
			{
				return _activejoint_dof;
			}

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
				std::map< std::string, unsigned int >::iterator iter = _linkMap.find(link_name);
				utils::Log(iter == _linkMap.end(), "찾고자 하는 이름을 갖는 링크가 존재하지 않습니다.", true);
				return _linkState[iter->second];
			}
			/// Link의 상태를 가지고 옵니다.
			LinkState& getLinkState(const unsigned int& link_num)
			{
				return _linkState[link_num];
			}
			/// Joint의 상태를 가지고 옵니다.
			JointState& getJointState(const std::string& joint_name)
			{
				std::map< std::string, unsigned int >::iterator iter = _jointMap.find(joint_name);
				utils::Log(iter == _jointMap.end(), "찾고자 하는 이름을 갖는 조인트가 존재하지 않습니다.", true);
				return _jointState[iter->second];
			}
			/// Link의 상태를 가지고 옵니다.
			JointState& getJointState(const unsigned int& joint_num)
			{
				return _jointState[joint_num];
			}
			/// Link의 상태를 가지고 옵니다.
			const LinkState& getLinkState(const std::string& link_name) const
			{
				std::map< std::string, unsigned int >::const_iterator iter = _linkMap.find(link_name);
				utils::Log(iter == _linkMap.end(), "찾고자 하는 이름을 갖는 링크가 존재하지 않습니다.", true);
				return _linkState[iter->second];
			}
			/// Link의 상태를 가지고 옵니다.
			const LinkState& getLinkState(const unsigned int& link_num) const
			{
				return _linkState[link_num];
			}
			/// Joint의 상태를 가지고 옵니다.
			const JointState& getJointState(const std::string& joint_name) const
			{
				std::map< std::string, unsigned int >::const_iterator iter = _jointMap.find(joint_name);
				utils::Log(iter == _jointMap.end(), "찾고자 하는 이름을 갖는 조인트가 존재하지 않습니다.", true);
				return _jointState[iter->second];
			}
			/// Link의 상태를 가지고 옵니다.
			const JointState& getJointState(const unsigned int& joint_num) const
			{
				return _jointState[joint_num];
			}

			/// Active joint인지 확인합니다.
			bool isActiveJoint(const std::string& joint_name)
			{
				std::map< std::string, unsigned int >::iterator iter = _jointMap.find(joint_name);
				utils::Log(iter == _jointMap.end(), "찾고자 하는 이름을 갖는 조인트가 존재하지 않습니다.", true);
				return _activeJoint[iter->second];
			}
			/// Active joint인지 확인합니다.
			bool isActiveJoint(const unsigned int& joint_num) const
			{
				return _activeJoint[joint_num];
			}

			/// Active joint의 리스트를 가지고 옵니다.
			const std::list< std::pair< std::string, unsigned int >>& getActiveJointList()
			{
				return _activeJointList;
			}
			/// Passive joint의 리스트를 가지고 옵니다.
			const std::list< std::pair< std::string, unsigned int >>& getPassiveJointList()
			{
				return _passiveJointList;
			}

			/// 리턴 행렬 또는 벡터의 크기를 정하기 위한 함수
			unsigned int getReturnDof(const System::RETURN_STATE& return_state);
			/// 리턴 행렬을 만들어줍니다.
			void makeReturnMatrix(Math::MatrixX& target, const Math::MatrixX& value, const unsigned int& row, const unsigned int& joint_num, const System::RETURN_STATE& return_state);

			/// Active joint의 q들을 설정합니다.
			void setActiveJoint_q(const Math::VectorX& q);
			/// Passive joint의 q들을 설정합니다.
			void setPassiveJoint_q(const Math::VectorX& q);
			/// Passive joint의 q들을 더합니다.
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