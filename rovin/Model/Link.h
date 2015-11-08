/**
*	\file	Link.h
*	\date	2015.11.04
*	\author	Keunjun (ckj@robotics.snu.ac.kr)
*	\brief	Link의 정보를 저장하는 클래스 정의
*/

#pragma once

#include <string>
#include <list>
#include <map>
#include <memory>

#include <Eigen/Dense>
#include <rovin/Math/Inertia.h>
#include <rovin/Math/LieGroup.h>
#include "GeometryInfo.h"

namespace rovin
{
	namespace Model
	{
		class GeometryInfo;
		class Joint;

		bool checkName(std::string);

		/**
		*	\class Link
		*	\brief Link를 생성하고 처리하는 클래스
		*/
		class Link
		{
			friend class Assembly;

		public:
			/// 기본 생성자
			Link() : _name(""), _inertia(), _visual(NULL), _collision(NULL), _material(""), _marker(), _joint() {}
			/**
			*	\warning 이름에 _BANNED_CHRACTERS 가 들어있는 경우 에러가 납니다.
			*	\brief	이름, 관성행렬, 보여지는 geometry, 충돌 geometry, 재료 이름을 차례로 받아서 생성한다.
			*/
			Link(const std::string name,
				const Math::Inertia& I = Math::Inertia(),
				const std::shared_ptr<GeometryInfo>& visual = NULL,
				const std::shared_ptr<GeometryInfo>& collision = NULL,
				const std::string material = ""
				) : _name((checkName(name) ? (name) : (assert(0), ""))), _inertia(I), _visual(visual), _collision(collision),
				_material(material), _marker(), _joint() {}

			/// 이름을 설정합니다.
			void setName(const std::string& name)
			{
				assert(checkName(name));
				_name = name;
			}
			/// 관성행렬을 설정합니다.
			void setInertia(const Math::Inertia& I)
			{
				_inertia = I;
			}
			/// 보여지는 geometry를 설정합니다.
			void setVisualGeometry(const std::shared_ptr<GeometryInfo>& visual)
			{
				_visual = visual;
			}
			/// 충돌 검사를 위한 geometry를 설정합니다.
			void setCollisionGeometry(const std::shared_ptr<GeometryInfo>& collision)
			{
				_collision = collision;
			}
			/// 재료 이름을 설정합니다.
			void setMaterial(const std::string& material)
			{
				_material = material;
			}

			/// 이름을 가져옵니다.
			const std::string& getName() const
			{
				return _name;
			}
			/// 관성행렬을 가져옵니다.
			const Math::Inertia& getInertia() const
			{
				return _inertia;
			}
			/// 보여지는 geometry를 가져옵니다.
			const std::shared_ptr<GeometryInfo>& getVisualGeometry()
			{
				return _visual;
			}
			/// 충돌 검사를 위한 geometry를 가져옵니다.
			const std::shared_ptr<GeometryInfo>& getCollisionGeometry()
			{
				return _collision;
			}
			/// 재료 이름을 가져옵니다.
			const std::string& getMaterial()
			{
				return _material;
			}

			/// Marker map을 가져옵니다.
			const std::map< std::string, Math::SE3 >& getMarkerMap() const
			{
				return _marker;
			}
			/**
			*	\warning 만약에 마커의 이름이 이미 있는 경우에는 옵션에 따라서 덮어쓰거나 건너뜁니다.
			*	\brief	Marker를 추가합니다.
			*/
			void addMarker(const std::string& marker_name, ///< Marker의 이름
				const Math::SE3& T, ///< Link frame으로부터 marker의 상대적인 위치
				bool overwrite = (false) ///< true: 덮어씁니다. false: 건너뜁니다.
				);
			/// Marker를 제거합니다.
			void deleteMarker(const std::string& marker_name ///< Marker의 이름
				);
			/// Marker를 모두 제거합니다.
			void clearMarker()
			{
				_marker.clear();
			}
			/**
			*	\warning 만약에 마커가 없는 경우에는 에러
			*	\brief	Marker의 상대적 위치 바꿉니다.
			*/
			void changeMarker(const std::string& marker_name, ///< Marker의 이름
				const Math::SE3& T ///< 바꾸고 싶은 위치
				);
			/**
			*	\return Marker의 상대적인 위치 SE3
			*	\warning 만약에 마커가 없는 경우에는 에러
			*	\brief	Marker의 상대적 위치를 찾아줍니다.
			*/
			const Math::SE3& findMarker(const std::string& marker_name ///< Marker의 이름
				) const;
			/// 인자로 받은 이름을 갖는 marker가 존재하는지 확인합니다.
			bool checkMarker(const std::string& marker_name ///< 확인하고 싶은 이름
				) const;

			/// Joint map을 가져옵니다.
			const std::list< std::tuple< std::string, std::weak_ptr<Joint>, Math::SE3 > >& getJointMap() const
			{
				return _joint;
			}

			/// 깊은 복사 - joint와 관련된 정보는 복사하지 않는다.
			std::shared_ptr<Link> copy()
			{
				Link *clone = new Link(_name, _inertia, (*_visual).copy(), (*_collision).copy(), _material);
				return std::shared_ptr<Link>(clone);
			}

		private:
			/// Assembly class에서만 접근 가능합니다. Joint를 이 link에 연결할 때 사용합니다.
			void addJoint(const std::string& joint_name, ///< Joint의 이름
				const std::weak_ptr<Joint>& joint_pointer, ///< 연결하고자 하는 joint의 포인터
				const Math::SE3& T ///< 링크 기준 frame에서 joint의 위치 SE3
				);
			/// Assembly class에서만 접근 가능합니다. Joint를 이 link에서 제거할 때 사용합니다.
			void deleteJoint(const std::string& joint_name ///< 연결을 끊고 싶은 joint의 포인터
				);

		private:
			std::string _name; ///< Link의 이름, identity로 쓰이므로 system 내부에서는 유일해야한다.
			Math::Inertia _inertia; ///< Inertia를 저장하고 있는 변수
			std::shared_ptr<GeometryInfo> _visual; ///< 보여지는 geometry를 저장하는 변수
			std::shared_ptr<GeometryInfo> _collision; ///< 충돌 검사를 위한 geometry를 저장하는 변수

			std::string _material; ///< Link의 물성을 나타내는 변수(재료 이름), 탄성계수와 마찰계수를 정의하기 위해서 쓰인다.

			std::map< std::string, Math::SE3 > _marker; ///< Link에 붙어있는 marker의 이름과 위치
			std::list< std::tuple< std::string, std::weak_ptr<Joint>, Math::SE3 > > _joint; ///< Link의 연결되어 있는 joint들의 리스트

		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		};

		static const int _NUM_OF_BANNED_CHARACTERS = 9;
		static const char _BANNED_CHRACTERS[_NUM_OF_BANNED_CHARACTERS] = { '\\', '/', ':', '*', '?', '\"', '<', '>', '|' };
		static bool checkName(std::string name)
		{
			int i;
			for (std::string::iterator pos = name.begin(); pos != name.end(); pos++)
			{
				for (i = 0; i < _NUM_OF_BANNED_CHARACTERS; i++)
				{
					if (*pos == _BANNED_CHRACTERS[i]) return false;
				}
			}
			return true;
		}
	}
}