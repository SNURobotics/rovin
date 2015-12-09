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
#include <vector>

#include <rovin/utils/Checker.h>
#include <rovin/Math/LieGroup.h>
#include <rovin/Math/Inertia.h>

#include "GeometryInfo.h"

namespace rovin
{
	namespace Model
	{
		class Link;
		class Joint;

		typedef std::shared_ptr< Model::Link > LinkPtr;

		/**
		*	\class Link
		*	\brief Link를 생성하고 처리하는 클래스
		*/
		class Link
		{
			friend class Assembly;

		public:
			/// 기본 생성자
			Link() : _name(""), _inertia(), _drawingShapes(std::vector<GeometryInfoPtr>()), _collisionShapes(std::vector<GeometryInfoPtr>()), _material(""), _marker() {}
			/**
			*	\warning 이름에 _BANNED_CHRACTERS 가 들어있는 경우 에러가 납니다.
			*	\brief	이름, 관성행렬, 보여지는 geometry, 충돌 geometry, 재료 이름을 차례로 받아서 생성한다.
			*/
			Link(const std::string name,
				const Math::Inertia& I = Math::Inertia(),
				const std::vector<GeometryInfoPtr>& visual = std::vector<GeometryInfoPtr>(),
				const std::vector<GeometryInfoPtr>& collision = std::vector<GeometryInfoPtr>(),
				const std::string material = "",
				const std::map< std::string, Math::SE3 >& marker = std::map< std::string, Math::SE3 >()
				) : _name((utils::checkName(name) ? (name) : (assert(0 && "링크의 이름으로 사용할 수 없는 이름이 들어왔습니다."), ""))), _inertia(I), _drawingShapes(visual), _collisionShapes(collision),
				_material(material), _marker(marker) {}

			/// 관성행렬을 설정합니다.
			void setInertia(const Math::Inertia& I)
			{
				_inertia = I;
			}
			/// 보여지는 geometry를 설정합니다.
			void addDrawingShapes(const std::shared_ptr<GeometryInfo>& shape)
			{
				_drawingShapes.push_back(shape);
			}
			/// 충돌 검사를 위한 geometry를 설정합니다.
			void addCollisionShapes(const std::shared_ptr<GeometryInfo>& collision)
			{
				_collisionShapes.push_back(collision);
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
			const std::vector<GeometryInfoPtr>& getDrawingShapes()
			{
				return _drawingShapes;
			}
			/// 충돌 검사를 위한 geometry를 가져옵니다.
			const std::vector<GeometryInfoPtr>& getCollisionShapes()
			{
				return _collisionShapes;
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
			void eraseMarker(const std::string& marker_name ///< Marker의 이름
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
			const Math::SE3& getMarker(const std::string& marker_name ///< Marker의 이름
				) const;
			/// 인자로 받은 이름을 갖는 marker가 존재하는지 확인합니다.
			bool isMarker(const std::string& marker_name ///< 확인하고 싶은 이름
				) const;

			/// 깊은 복사 - joint와 관련된 정보는 복사하지 않는다.
			LinkPtr copy() const
			{
				LinkPtr ret(new Link(_name, _inertia, std::vector<GeometryInfoPtr>(), std::vector<GeometryInfoPtr>(), _material, _marker));
				for (unsigned int i = 0; i < _drawingShapes.size(); i++)
					ret->addDrawingShapes(_drawingShapes[i]->copy());
				for (unsigned int i = 0; i < _collisionShapes.size(); i++)
					ret->addCollisionShapes(_drawingShapes[i]->copy());
				return ret;
				//return LinkPtr(new Link(_name, _inertia, (*_drawingShapes).copy(), (*_collisionShapes).copy(), _material, _marker));
			}

		private:
			std::string _name; ///< Link의 이름, identity로 쓰이므로 system 내부에서는 유일해야한다.
			Math::Inertia _inertia; ///< Inertia를 저장하고 있는 변수
			std::vector<GeometryInfoPtr> _drawingShapes; ///< 보여지는 geometry를 저장하는 변수
			std::vector<GeometryInfoPtr> _collisionShapes; ///< 충돌 검사를 위한 geometry를 저장하는 변수

			std::string _material; ///< Link의 물성을 나타내는 변수(재료 이름), 탄성계수와 마찰계수를 정의하기 위해서 쓰인다.

			std::map< std::string, Math::SE3 > _marker; ///< Link에 붙어있는 marker의 이름과 위치

		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
		};
	}
}