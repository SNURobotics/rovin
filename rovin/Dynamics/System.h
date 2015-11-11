/**
*	\file	System.h
*	\date	2015.11.10
*	\author	Keunjun (ckj@robotics.snu.ac.kr)
*	\brief	System�� ������ �����ϴ� Ŭ���� ����
*/

#pragma once

#include <string>
#include <memory>
#include <map>

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
		*	\brief System�� �����ϰ� ó���ϴ� Ŭ����
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

				unsigned int _slink; ///< ���� ��ũ�� ��ȣ
				unsigned int _elink; ///< ���� ��ũ�� ��ȣ
				unsigned int _joint; ///< ��������ִ� ����Ʈ�� ��ȣ
				bool _direction; ///< true �� ���� mount���� action, false�� ��쿡�� action���� mount
				Math::SE3 _sj; ///< ���� ��ũ �����ӿ��� ����Ʈ ��ũ ������
				Math::SE3 _je; ///< ����Ʈ ��ũ �����ӿ��� ���� ��ũ�� ������
			};

			/// ������
			System(const Model::Assembly& model, const std::string& baselink);

			/// �Ҹ���
			~System()
			{
				delete _link;
				delete _joint;

				delete _activejoint;

				delete _linkptr;
				delete _jointptr;

				delete _connectionlist;

				delete _tree;
				delete _trace;
			}
			
			/// ��ũ ��ȣ�� ���ɴϴ�.
			unsigned int getLinkNum(const std::string& linkname)
			{
				std::map< std::string, unsigned int >::iterator iter = _linkmap.find(linkname);
				utils::Log(iter == _linkmap.end(), "ã�����ϴ� �̸��� ���� ��ũ�� �������� �ʽ��ϴ�.", true);
				return iter->second;
			}
			/// ����Ʈ ��ȣ�� ���ɴϴ�.
			unsigned int getJointNum(const std::string& jointname)
			{
				std::map< std::string, unsigned int >::iterator iter = _jointmap.find(jointname);
				utils::Log(iter == _jointmap.end(), "ã�����ϴ� �̸��� ���� ����Ʈ�� �������� �ʽ��ϴ�.", true);
				return iter->second;
			}

			void stateLock(Dynamics::State& state)
			{
				_statelock = true;
				_state = &state;

				_linkstate = new State::LinkState*[_num_link];
				_jointstate = new State::JointState*[_num_joint];

				for (int i = 0; i < _num_link; i++)
				{
					_linkstate[i] = &_state->getLinkState(_link[i]);
				}
				for (int i = 0; i < _num_joint; i++)
				{
					_jointstate[i] = &_state->getJointState(_link[i]);
				}

				_activejoint = new bool[_num_joint];
				const std::list< std::string > activejoint = _state->getActiveJointList();
				for (std::list< std::string >::const_iterator iter = activejoint.begin(); iter != activejoint.end(); iter++)
				{
					_activejoint[getJointNum(*iter)] = true;
				}
			}
			void stateUnlock()
			{
				_statelock = false;
				_state = NULL;

				delete *_linkstate;
				delete *_jointstate;

				delete _activejoint;
			}

			void stateActiveJointUpdate()
			{
				utils::Log(!_statelock, "System�� state�� lock�Ǿ� �־�� �մϴ�", true);

				delete _activejoint;
				_activejoint = new bool[_num_joint];
				const std::list< std::string > activejoint = _state->getActiveJointList();
				for (std::list< std::string >::const_iterator iter = activejoint.begin(); iter != activejoint.end(); iter++)
				{
					_activejoint[getJointNum(*iter)] = true;
				}
			}
		private:
			const Model::Assembly* _model;
			std::string _baselink;

			unsigned int _num_link;
			unsigned int _num_joint;

			std::string *_link;
			std::string *_joint;

			std::shared_ptr<Model::Link> *_linkptr;
			std::shared_ptr<Model::Joint> *_jointptr;

			std::map< std::string, unsigned int > _linkmap;
			std::map< std::string, unsigned int > _jointmap;

			std::list< _CONN > *_connectionlist;

			unsigned _root;
			std::list< _CONN > *_tree;
			std::list< _CONN > *_trace;

			std::list< std::list< _CONN >> _closedloop;

			bool _statelock;
			State* _state;

			State::LinkState **_linkstate;
			State::JointState **_jointstate;

			bool *_activejoint;
		};
	}
}