/**
*	\file	System.h
*	\date	2015.11.10
*	\author	Keunjun (ckj@robotics.snu.ac.kr)
*	\brief	System의 정보를 저장하는 클래스 정의
*/

#pragma once

#include <string>
#include <memory>
#include <map>
#include <vector>

#include <rovin/utils/Diagnostic.h>
#include <rovin/Math/LieGroup.h>
#include <rovin/Model/Assembly.h>
#include <rovin/Model/Link.h>
#include <rovin/Model/Joint.h>

#include "State.h"

namespace rovin
{
	namespace Dynamics
	{
		/**
		*	\class System
		*	\brief System를 생성하고 처리하는 클래스
		*/
		class System
		{
		public:
			class _CONN
			{
			public:
				_CONN(const unsigned int& slink, const Math::SE3& sj, const unsigned int& joint, const Math::SE3& je,
					const unsigned int& elink, const bool direction) :
					_slink(slink), _sj(sj), _joint(joint), _je(je), _elink(elink), _direction(direction) {}

				_CONN flip()
				{
					return _CONN(_elink, _je.inverse(), _joint, _sj.inverse(), _slink, !_direction);
				}

				unsigned int _slink; ///< 시작 링크의 번호
				unsigned int _elink; ///< 다음 링크의 번호
				unsigned int _joint; ///< 연결시켜주는 조인트의 번호
				bool _direction; ///< true 인 경우는 mount에서 action, false인 경우에는 action에서 mount
				Math::SE3 _sj; ///< 시작 링크 프레임에서 조인트 링크 프레임
				Math::SE3 _je; ///< 조인트 링크 프레임에서 다음 링크의 프레임
			};

			/// 생성자
			System(const Model::Assembly& model, const std::string& baselink);
			/// 소멸자
			~System() {}
			
			/// 링크 번호를 얻어옵니다.
			unsigned int getLinkNum(const std::string& linkname)
			{
				std::map< std::string, unsigned int >::iterator iter = _linkmap.find(linkname);
				utils::Log(iter == _linkmap.end(), "찾고자하는 이름을 갖는 링크는 존재하지 않습니다.", true);
				return iter->second;
			}
			/// 조인트 번호를 얻어옵니다.
			unsigned int getJointNum(const std::string& jointname)
			{
				std::map< std::string, unsigned int >::iterator iter = _jointmap.find(jointname);
				utils::Log(iter == _jointmap.end(), "찾고자하는 이름을 갖는 조인트는 존재하지 않습니다.", true);
				return iter->second;
			}

			void ConnectState(Dynamics::State& state)
			{
				if (_connectstate) DisconnectState();

				_connectstate = true;
				_state = &state;

				_linkstate.resize(_num_link);
				_jointstate.resize(_num_joint);

				for (unsigned int i = 0; i < _num_link; i++)
				{
					_linkstate[i] = &_state->getLinkState(_link[i]);
				}
				for (unsigned int i = 0; i < _num_joint; i++)
				{
					_jointstate[i] = &_state->getJointState(_link[i]);
				}

				_activejoint.resize(_num_joint);
				_activejoint_index.resize(_num_link);
				unsigned int temp_index = 0;
				const std::list< std::string > activejoint = _state->getActiveJointList();
				for (std::list< std::string >::const_iterator iter = activejoint.begin(); iter != activejoint.end(); iter++)
				{
					unsigned int temp_joint = getJointNum(*iter);
					_activejoint[temp_joint] = true;
					_activejoint_index[temp_joint] = temp_index;
					temp_index += _jointstate[temp_joint]->dof;
				}
			}
			void DisconnectState()
			{
				_connectstate = false;
				_state = NULL;
				_linkstate.clear();
				_jointstate.clear();
				_activejoint.clear();
				_activejoint_index.clear();
			}

			void stateActiveJointUpdate()
			{
				utils::Log(!_connectstate, "System에 state가 연결되어 있어야 합니다", true);

				_activejoint.clear();
				_activejoint_index.clear();
				_activejoint.resize(_num_joint);
				_activejoint_index.resize(_num_link);
				unsigned int temp_index = 0;
				const std::list< std::string > activejoint = _state->getActiveJointList();
				for (std::list< std::string >::const_iterator iter = activejoint.begin(); iter != activejoint.end(); iter++)
				{
					unsigned int temp_joint = getJointNum(*iter);
					_activejoint[temp_joint] = true;
					_activejoint_index[temp_joint] = temp_index;
					temp_index += _jointstate[temp_joint]->dof;
				}
			}

			/// Closed Loop 조건 식의 값을 구합니다.
			Math::VectorX Closedloop_Constraint_Function(State& state);
			/// Closed Loop 조건 식의 값을 구합니다.
			Math::VectorX Closedloop_Constraint_Function() const;
			/// Closed Loop 조건 식의 Jacobian 행렬을 구합니다.
			Math::MatrixX Closedloop_Constraint_Jacobian(State& state);
			/// Closed Loop 조건 식의 Jacobian 행렬을 구합니다.
			Math::MatrixX Closedloop_Constraint_Jacobian() const;
		private:
			const Model::Assembly* _model;
			std::string _baselink;

			unsigned int _num_link;
			unsigned int _num_joint;

			std::vector< std::string > _link;
			std::vector< std::string > _joint;

			std::vector< std::shared_ptr< Model::Link >> _linkptr;
			std::vector< std::shared_ptr< Model::Joint >> _jointptr;

			std::map< std::string, unsigned int > _linkmap;
			std::map< std::string, unsigned int > _jointmap;

			std::vector< std::list< _CONN >> _connectionlist;

			unsigned _root;
			std::vector< std::list< _CONN >> _tree;
			std::vector< std::list< _CONN >> _trace;

			std::list< std::list< _CONN >> _closedloop;

			bool _connectstate;
			State* _state;

			std::vector< State::LinkState* > _linkstate;
			std::vector< State::JointState* > _jointstate;

			std::vector< bool > _activejoint;

			std::vector< int > _activejoint_index;
		};
	}
}