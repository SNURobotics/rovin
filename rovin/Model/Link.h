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
		*	\brief Link�� �����ϰ� ó���ϴ� Ŭ����
		*/
		class Link
		{
			friend class Assembly;

		public:
			/// �⺻ ������
			Link() : _name(""), _inertia(), _visual(NULL), _collision(NULL), _material(""), _marker(), _joint() {}
			/**
			*	\warning �̸��� _BANNED_CHRACTERS �� ����ִ� ��� ������ ���ϴ�.
			*	\brief	�̸�, �������, �������� geometry, �浹 geometry, ��� �̸��� ���ʷ� �޾Ƽ� �����Ѵ�.
			*/
			Link(const std::string name,
				const Math::Inertia& I = Math::Inertia(),
				const std::shared_ptr<GeometryInfo>& visual = NULL,
				const std::shared_ptr<GeometryInfo>& collision = NULL,
				const std::string material = ""
				) : _name((checkName(name) ? (name) : (assert(0), ""))), _inertia(I), _visual(visual), _collision(collision),
				_material(material), _marker(), _joint() {}

			/// �̸��� �����մϴ�.
			void setName(const std::string& name)
			{
				assert(checkName(name));
				_name = name;
			}
			/// ��������� �����մϴ�.
			void setInertia(const Math::Inertia& I)
			{
				_inertia = I;
			}
			/// �������� geometry�� �����մϴ�.
			void setVisualGeometry(const std::shared_ptr<GeometryInfo>& visual)
			{
				_visual = visual;
			}
			/// �浹 �˻縦 ���� geometry�� �����մϴ�.
			void setCollisionGeometry(const std::shared_ptr<GeometryInfo>& collision)
			{
				_collision = collision;
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
			const std::shared_ptr<GeometryInfo>& getVisualGeometry()
			{
				return _visual;
			}
			/// �浹 �˻縦 ���� geometry�� �����ɴϴ�.
			const std::shared_ptr<GeometryInfo>& getCollisionGeometry()
			{
				return _collision;
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
			void deleteMarker(const std::string& marker_name ///< Marker�� �̸�
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
			const Math::SE3& findMarker(const std::string& marker_name ///< Marker�� �̸�
				) const;
			/// ���ڷ� ���� �̸��� ���� marker�� �����ϴ��� Ȯ���մϴ�.
			bool checkMarker(const std::string& marker_name ///< Ȯ���ϰ� ���� �̸�
				) const;

			/// Joint map�� �����ɴϴ�.
			const std::list< std::tuple< std::string, std::weak_ptr<Joint>, Math::SE3 > >& getJointMap() const
			{
				return _joint;
			}

			/// ���� ���� - joint�� ���õ� ������ �������� �ʴ´�.
			std::shared_ptr<Link> copy()
			{
				Link *clone = new Link(_name, _inertia, (*_visual).copy(), (*_collision).copy(), _material);
				return std::shared_ptr<Link>(clone);
			}

		private:
			/// Assembly class������ ���� �����մϴ�. Joint�� �� link�� ������ �� ����մϴ�.
			void addJoint(const std::string& joint_name, ///< Joint�� �̸�
				const std::weak_ptr<Joint>& joint_pointer, ///< �����ϰ��� �ϴ� joint�� ������
				const Math::SE3& T ///< ��ũ ���� frame���� joint�� ��ġ SE3
				);
			/// Assembly class������ ���� �����մϴ�. Joint�� �� link���� ������ �� ����մϴ�.
			void deleteJoint(const std::string& joint_name ///< ������ ���� ���� joint�� ������
				);

		private:
			std::string _name; ///< Link�� �̸�, identity�� ���̹Ƿ� system ���ο����� �����ؾ��Ѵ�.
			Math::Inertia _inertia; ///< Inertia�� �����ϰ� �ִ� ����
			std::shared_ptr<GeometryInfo> _visual; ///< �������� geometry�� �����ϴ� ����
			std::shared_ptr<GeometryInfo> _collision; ///< �浹 �˻縦 ���� geometry�� �����ϴ� ����

			std::string _material; ///< Link�� ������ ��Ÿ���� ����(��� �̸�), ź������� ��������� �����ϱ� ���ؼ� ���δ�.

			std::map< std::string, Math::SE3 > _marker; ///< Link�� �پ��ִ� marker�� �̸��� ��ġ
			std::list< std::tuple< std::string, std::weak_ptr<Joint>, Math::SE3 > > _joint; ///< Link�� ����Ǿ� �ִ� joint���� ����Ʈ

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