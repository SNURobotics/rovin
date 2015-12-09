/**
*	\file	Link.h
*	\date	2015.11.04
*	\author	Keunjun (ckj@robotics.snu.ac.kr)
*	\brief	Link�� ������ �����ϴ� Ŭ���� ����
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
		*	\brief Link�� �����ϰ� ó���ϴ� Ŭ����
		*/
		class Link
		{
			friend class Assembly;

		public:
			/// �⺻ ������
			Link() : _name(""), _inertia(), _drawingShapes(std::vector<GeometryInfoPtr>()), _collisionShapes(std::vector<GeometryInfoPtr>()), _material(""), _marker() {}
			/**
			*	\warning �̸��� _BANNED_CHRACTERS �� ����ִ� ��� ������ ���ϴ�.
			*	\brief	�̸�, �������, �������� geometry, �浹 geometry, ��� �̸��� ���ʷ� �޾Ƽ� �����Ѵ�.
			*/
			Link(const std::string name,
				const Math::Inertia& I = Math::Inertia(),
				const std::vector<GeometryInfoPtr>& visual = std::vector<GeometryInfoPtr>(),
				const std::vector<GeometryInfoPtr>& collision = std::vector<GeometryInfoPtr>(),
				const std::string material = "",
				const std::map< std::string, Math::SE3 >& marker = std::map< std::string, Math::SE3 >()
				) : _name((utils::checkName(name) ? (name) : (assert(0 && "��ũ�� �̸����� ����� �� ���� �̸��� ���Խ��ϴ�."), ""))), _inertia(I), _drawingShapes(visual), _collisionShapes(collision),
				_material(material), _marker(marker) {}

			/// ��������� �����մϴ�.
			void setInertia(const Math::Inertia& I)
			{
				_inertia = I;
			}
			/// �������� geometry�� �����մϴ�.
			void addDrawingShapes(const std::shared_ptr<GeometryInfo>& shape)
			{
				_drawingShapes.push_back(shape);
			}
			/// �浹 �˻縦 ���� geometry�� �����մϴ�.
			void addCollisionShapes(const std::shared_ptr<GeometryInfo>& collision)
			{
				_collisionShapes.push_back(collision);
			}
			/// ��� �̸��� �����մϴ�.
			void setMaterial(const std::string& material)
			{
				_material = material;
			}

			/// �̸��� �����ɴϴ�.
			const std::string& getName() const
			{
				return _name;
			}
			/// ��������� �����ɴϴ�.
			const Math::Inertia& getInertia() const
			{
				return _inertia;
			}
			/// �������� geometry�� �����ɴϴ�.
			const std::vector<GeometryInfoPtr>& getDrawingShapes()
			{
				return _drawingShapes;
			}
			/// �浹 �˻縦 ���� geometry�� �����ɴϴ�.
			const std::vector<GeometryInfoPtr>& getCollisionShapes()
			{
				return _collisionShapes;
			}
			/// ��� �̸��� �����ɴϴ�.
			const std::string& getMaterial()
			{
				return _material;
			}

			/// Marker map�� �����ɴϴ�.
			const std::map< std::string, Math::SE3 >& getMarkerMap() const
			{
				return _marker;
			}
			/**
			*	\warning ���࿡ ��Ŀ�� �̸��� �̹� �ִ� ��쿡�� �ɼǿ� ���� ����ų� �ǳʶݴϴ�.
			*	\brief	Marker�� �߰��մϴ�.
			*/
			void addMarker(const std::string& marker_name, ///< Marker�� �̸�
				const Math::SE3& T, ///< Link frame���κ��� marker�� ������� ��ġ
				bool overwrite = (false) ///< true: ����ϴ�. false: �ǳʶݴϴ�.
				);
			/// Marker�� �����մϴ�.
			void eraseMarker(const std::string& marker_name ///< Marker�� �̸�
				);
			/// Marker�� ��� �����մϴ�.
			void clearMarker()
			{
				_marker.clear();
			}
			/**
			*	\warning ���࿡ ��Ŀ�� ���� ��쿡�� ����
			*	\brief	Marker�� ����� ��ġ �ٲߴϴ�.
			*/
			void changeMarker(const std::string& marker_name, ///< Marker�� �̸�
				const Math::SE3& T ///< �ٲٰ� ���� ��ġ
				);
			/**
			*	\return Marker�� ������� ��ġ SE3
			*	\warning ���࿡ ��Ŀ�� ���� ��쿡�� ����
			*	\brief	Marker�� ����� ��ġ�� ã���ݴϴ�.
			*/
			const Math::SE3& getMarker(const std::string& marker_name ///< Marker�� �̸�
				) const;
			/// ���ڷ� ���� �̸��� ���� marker�� �����ϴ��� Ȯ���մϴ�.
			bool isMarker(const std::string& marker_name ///< Ȯ���ϰ� ���� �̸�
				) const;

			/// ���� ���� - joint�� ���õ� ������ �������� �ʴ´�.
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
			std::string _name; ///< Link�� �̸�, identity�� ���̹Ƿ� system ���ο����� �����ؾ��Ѵ�.
			Math::Inertia _inertia; ///< Inertia�� �����ϰ� �ִ� ����
			std::vector<GeometryInfoPtr> _drawingShapes; ///< �������� geometry�� �����ϴ� ����
			std::vector<GeometryInfoPtr> _collisionShapes; ///< �浹 �˻縦 ���� geometry�� �����ϴ� ����

			std::string _material; ///< Link�� ������ ��Ÿ���� ����(��� �̸�), ź������� ��������� �����ϱ� ���ؼ� ���δ�.

			std::map< std::string, Math::SE3 > _marker; ///< Link�� �پ��ִ� marker�� �̸��� ��ġ

		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
		};
	}
}