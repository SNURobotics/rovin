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

namespace rovin
{
	namespace Dynamics
	{
		class State;

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

			enum RETURN_STATE
			{
				SYSTEMJOINT,
				WHOLEJOINT,
				ACTIVEJOINT,
				PASSIVEJOINT
			};

			/// 생성자
			System(const std::shared_ptr< Model::Assembly >& model, const std::string& baselink);
			/// 소멸자
			~System()
			{
				_model->UNLOCK();
			}

			/// 현재 system에 연결된 assembly를 가져옵니다.
			const Model::Assembly& getAssembly() const
			{
				return *_model;
			}
			/// 링크 리스트를 가져옵니다.
			const std::vector< std::string >& getLinkList() const
			{
				return _link;
			}
			/// 링크 리스트를 가져옵니다.
			const std::vector< std::string >& getJointList() const
			{
				return _joint;
			}
			
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

			/// Closed Loop 조건 식의 값을 구합니다.
			Math::VectorX Closedloop_Constraint_Function(State& state);
			/// Closed Loop 조건 식의 Jacobian 행렬을 구합니다.
			Math::MatrixX Closedloop_Constraint_Jacobian(State& state, const RETURN_STATE& return_state);
			/// Closed Loop 조건을 풀어줍니다. Active joint는 상수로 passive joint를 변수로 두고 조건을 맞는 변수 값을 구합니다.
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