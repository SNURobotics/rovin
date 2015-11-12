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
#include <vector>

#include <rovin/utils/Diagnostic.h>
#include <rovin/Math/LieGroup.h>
#include <rovin/Model/Assembly.h>
#include <rovin/Model/Link.h>
#include <rovin/Model/Joint.h>

namespace rovin
{
	namespace Dynamics
	{
		class State;

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

			enum RETURN_STATE
			{
				SYSTEMJOINT,
				WHOLEJOINT,
				ACTIVEJOINT,
				PASSIVEJOINT
			};

			/// ������
			System(const std::shared_ptr< Model::Assembly >& model, const std::string& baselink);
			/// �Ҹ���
			~System()
			{
				_model->UNLOCK();
			}

			/// ���� system�� ����� assembly�� �����ɴϴ�.
			const Model::Assembly& getAssembly() const
			{
				return *_model;
			}
			/// ��ũ ����Ʈ�� �����ɴϴ�.
			const std::vector< std::string >& getLinkList() const
			{
				return _link;
			}
			/// ��ũ ����Ʈ�� �����ɴϴ�.
			const std::vector< std::string >& getJointList() const
			{
				return _joint;
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

			/// Closed Loop ���� ���� ���� ���մϴ�.
			Math::VectorX Closedloop_Constraint_Function(State& state);
			/// Closed Loop ���� ���� Jacobian ����� ���մϴ�.
			Math::MatrixX Closedloop_Constraint_Jacobian(State& state, const RETURN_STATE& return_state);
			/// Closed Loop ������ Ǯ���ݴϴ�. Active joint�� ����� passive joint�� ������ �ΰ� ������ �´� ���� ���� ���մϴ�.
			void Solve_Closedloop_Constraint(State& state);

		private:
			std::shared_ptr< Model::Assembly > _model;
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

			unsigned _baseLinkIndex;
			std::vector< std::list< _CONN >> _tree;
			std::vector< std::list< _CONN >> _trace;

			std::list< std::list< _CONN >> _closedloop;
		};
	}
}