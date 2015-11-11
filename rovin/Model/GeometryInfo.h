/**
*	\file	srGeometryInfo.h
*	\date	2015.11.04
*	\author	Keunjun (ckj@robotics.snu.ac.kr)
*	\brief	Geometry�� ������ �����ϴ� Ŭ�������� ����
*/

#pragma once

#include <string>
#include <list>
#include <memory>

#include <rovin/Math/Constant.h>
#include <rovin/Math/LieGroup.h>

namespace rovin
{
	namespace Model
	{
		/**
		*	\class GeometryInfo
		*	\brief Geometry�� Type�� �����ϰ� ��ǥ�ϴ� ���� Ŭ����
		*/
		class GeometryInfo
		{
		public:
			/// Geometry type�� ����
			enum GEOMETRY_TYPE
			{
				_BOX,		///< Box ����
				_SPHERE,	///< Sphere ��
				_CAPSULE,	///< Capsule ĸ��
				_CYLINDER,	///< Cylinder ����
				_MESH,		///< Mesh mesh�� ������ ����
				_USERMODEL	///< UserModel ����ڰ� ���� ����� ��
			};

			/// ������
			GeometryInfo(const GEOMETRY_TYPE& Type, ///< Type
				const Math::SE3& T = (Math::SE3()), ///< Frame��ġ
				const Math::Vector4& Color = (Math::Vector4(-1, -1, -1, -1)) ///< (R, G, B, Alpha) ��
				) : _Type(Type), _T(T), _Color(Color) {}

			/// �Ҹ���, ���� Ŭ������ ����� �ݴϴ�.
			virtual ~GeometryInfo() = 0;

			/// Type�� �����մϴ�.
			void setType(const GEOMETRY_TYPE& Type)
			{
				_Type = Type;
			}
			/// Frame�� ��ġ�� �����մϴ�.
			void setFrame(const Math::SE3& T)
			{
				_T = T;
			}
			/// (R, G, B, Alpha) ���� �����մϴ�.
			void setColor(const Math::Real& R, ///< Red
				const Math::Real& G, ///< Green
				const Math::Real& B, ///< Blue
				const Math::Real& Alpha = (0.0) /// Alpha
				)
			{
				_Color << R, G, B, Alpha;
			}
			/// (R, G, B, Alpha) ���� �����մϴ�.
			void setColor(const Math::Vector4& Color // �� [R; G; B; Alpha]
				)
			{
				_Color = Color;
			}

			/// ���� geometry�� type�� ������ �ɴϴ�.
			const GEOMETRY_TYPE& getType() const
			{
				return _Type;
			}
			/**
			*	\return Math::Vector4 [R; G; B; Alpha]
			*	\brief ���� ���� �Ǿ��ִ� ���� �˷��ݴϴ�.
			*/
			const Math::Vector4& getColor() const
			{
				return _Color;
			}
			/**
			*	\return SE3 Frame
			*	\brief ���� ���� �Ǿ� �ִ� Frame�� �˷��ݴϴ�.
			*/
			const Math::SE3& getFrame() const
			{
				return _T;
			}

			/// Geometry ���� ����, �ڽ� Ŭ������ ������ copy�Լ� ������ �ʿ��մϴ�.
			virtual std::shared_ptr<GeometryInfo> copy() const = 0;

		private:
			GEOMETRY_TYPE _Type; ///< Geometry type�� �����ϴ� ����
			Math::SE3 _T; ///< Frame�� ��ġ�� �����ϰ� �ִ� ����
			Math::Vector4 _Color; ///< R, G, B, Alpha ���� �����ϴ� ����

		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		};

		/**
		*	\class Box
		*	\brief Geometry box�� ������ �����ϴ� Ŭ����, GEOMETRY_TYPE = _BOX
		*/
		class Box :public GeometryInfo
		{
		public:
			/// �⺻ ������, ����=0, ����=0, ����=0�� �ʱ�ȭ�˴ϴ�.
			Box(const Math::SE3& T = (Math::SE3()), ///< Frame��ġ
				const Math::Vector4& Color = (Math::Vector4(-1, -1, -1, -1)) ///< (R, G, B, Alpha) ��
				) : GeometryInfo(GeometryInfo::_BOX, T, Color),
				_width(0.0), _height(0.0), _depth(0.0) {}
			/// �Է����� ���� ����, ����, ���̷� �ʱ�ȭ�˴ϴ�.
			Box(const Math::Real& width, ///< ����
				const Math::Real& depth, ///< ����
				const Math::Real& height, ///< ����
				const Math::SE3& T = (Math::SE3()), ///< Frame��ġ
				const Math::Vector4& Color = (Math::Vector4(-1, -1, -1, -1)) ///< (R, G, B, Alpha) ��
				) : GeometryInfo(GeometryInfo::_BOX, T, Color),
				_width(width), _height(height), _depth(depth) {}
			/// �Է����� ���� config�� �̿��Ͽ� �ʱ�ȭ�� �մϴ�.
			Box(const Math::Vector3& config, ///< [����; ����; ����]
				const Math::SE3& T = (Math::SE3()), ///< Frame��ġ
				const Math::Vector4& Color = (Math::Vector4(-1, -1, -1, -1)) ///< (R, G, B, Alpha) ��
				) : GeometryInfo(GeometryInfo::_BOX, T, Color),
				_width(config(0)), _height(config(1)), _depth(config(2)) {}

			/// �Ҹ���
			~Box() {}

			/// �Է����� ���� ����, ����, ���̷� ���� �ٲߴϴ�.
			void setDimension(const Math::Real& width, ///< ����
				const Math::Real& depth, ///< ����
				const Math::Real& height ///< ����
				);
			/// �Է����� ���� config�� ����, ����, ���̸� �ٲ��ݴϴ�.
			void setDimension(const Math::Vector3& config ///< [����; ����; ����]
				);
			/// �Է����� ���� ���� �Ѻ��� ���̷� �ϴ� ������ü�� ����ϴ�.
			void setCube(const Math::Real& length ///< ������ü�� �Ѻ��� ����
				);

			/**
			*	\return Math::Vector3 [����; ����; ����]
			*	\brief ���� ���� �Ǿ��ִ� ����, ����, ���̸� �˷��ݴϴ�.
			*/
			Math::Vector3 getDimension() const;

			// ���� ����
			std::shared_ptr<GeometryInfo> copy() const;

		private:
			Math::Real _width, _depth, _height;
		};

		/**
		*	\class Sphere
		*	\brief Geometry sphere�� ������ �����ϴ� Ŭ����, GEOMETRY_TYPE = _SPHERE
		*/
		class Sphere :public GeometryInfo
		{
		public:
			/// �⺻ ������, ������=0�� �ʱ�ȭ�˴ϴ�.
			Sphere(const Math::SE3& T = (Math::SE3()), ///< Frame��ġ
				const Math::Vector4& Color = (Math::Vector4(-1, -1, -1, -1)) ///< (R, G, B, Alpha) ��
				) : GeometryInfo(GeometryInfo::_SPHERE, T, Color),
				_radius(0.0) {}
			/// �Է����� ���� ���������� �ʱ�ȭ�˴ϴ�.
			Sphere(const Math::Real& radius, ///< ������
				const Math::SE3& T = (Math::SE3()), ///< Frame��ġ
				const Math::Vector4& Color = (Math::Vector4(-1, -1, -1, -1)) ///< (R, G, B, Alpha) ��
				) : GeometryInfo(GeometryInfo::_SPHERE, T, Color),
				_radius(radius) {}

			/// �Ҹ���
			~Sphere() {}

			/// �Է����� ���� radius�� ������ ���� �ٲߴϴ�.
			void setRadius(const Math::Real& radius ///< ������
				);

			/**
			*	\return ������
			*	\brief ���� ���� �Ǿ��ִ� ������ ���� �˷��ݴϴ�.
			*/
			const Math::Real& getRadius() const;

			// ���� ����
			std::shared_ptr<GeometryInfo> copy() const;

		private:
			Math::Real _radius;
		};

		/**
		*	\class Capsule
		*	\brief Geometry capsule�� ������ �����ϴ� Ŭ����, GEOMETRY_TYPE = _CAPSULE
		*/
		class Capsule :public GeometryInfo
		{
		public:
			/// �⺻ ������, ������=0, ����=0�� �ʱ�ȭ�˴ϴ�.
			Capsule(const Math::SE3& T = (Math::SE3()), ///< Frame��ġ
				const Math::Vector4& Color = (Math::Vector4(-1, -1, -1, -1)) ///< (R, G, B, Alpha) ��
				) : GeometryInfo(GeometryInfo::_CAPSULE, T, Color),
				_radius(0.0), _height(0.0) {}
			/// �Է����� ���� �������� ���̷� �ʱ�ȭ�˴ϴ�.
			Capsule(const Math::Real& radius, ///< ������
				const Math::Real& height, ///< ����
				const Math::SE3& T = (Math::SE3()), ///< Frame��ġ
				const Math::Vector4& Color = (Math::Vector4(-1, -1, -1, -1)) ///< (R, G, B, Alpha) ��
				) : GeometryInfo(GeometryInfo::_CAPSULE, T, Color),
				_radius(radius), _height(height) {}

			/// �Ҹ���
			~Capsule() {}

			/// �Է����� ���� ������, ���̷� ���� �ٲߴϴ�.
			void setDimension(const Math::Real& radius, ///< ����
				const Math::Real& height ///< ����
				);
			/// �Է����� ���� config�� ������, ���̸� �ٲ��ݴϴ�.
			void setDimension(const Math::Vector2& config ///< [������: ����]
				);

			/**
			*	\return Math::Vector2 [������; ����]
			*	\brief ���� ���� �Ǿ��ִ� ������, ���̸� �˷��ݴϴ�.
			*/
			Math::Vector2 getDimension() const;

			// ���� ����
			std::shared_ptr<GeometryInfo> copy() const;

		private:
			Math::Real _radius, _height;
		};

		/**
		*	\class Cylinder
		*	\brief Geometry cylinder�� ������ �����ϴ� Ŭ����, GEOMETRY_TYPE = _CYLINDER
		*/
		class Cylinder :public GeometryInfo
		{
		public:
			/// �⺻ ������, ������=0, ����=0�� �ʱ�ȭ�˴ϴ�.
			Cylinder(const Math::SE3& T = (Math::SE3()), ///< Frame��ġ
				const Math::Vector4& Color = (Math::Vector4(-1, -1, -1, -1)) ///< (R, G, B, Alpha) ��
				) : GeometryInfo(GeometryInfo::_CYLINDER, T, Color),
				_radius(0.0), _height(0.0) {}
			/// �Է����� ���� �������� ���̷� �ʱ�ȭ�˴ϴ�.
			Cylinder(const Math::Real& radius, ///< ������
				const Math::Real& height, ///< ����
				const Math::SE3& T = (Math::SE3()), ///< Frame��ġ
				const Math::Vector4& Color = (Math::Vector4(-1, -1, -1, -1)) ///< (R, G, B, Alpha) ��
				) : GeometryInfo(GeometryInfo::_CYLINDER, T, Color),
				_radius(radius), _height(height) {}

			/// �Ҹ���
			~Cylinder() {}

			/// �Է����� ���� ������, ���̷� ���� �ٲߴϴ�.
			void setDimension(const Math::Real& radius, ///< ����
				const Math::Real& height ///< ����
				);
			/// �Է����� ���� config�� ������, ���̸� �ٲ��ݴϴ�.
			void setDimension(const Math::Vector2& config ///< [������: ����]
				);

			/**
			*	\return Math::Vector2 [������; ����]
			*	\brief ���� ���� �Ǿ��ִ� ������, ���̸� �˷��ݴϴ�.
			*/
			Math::Vector2 getDimension() const;

			// ���� ����
			std::shared_ptr<GeometryInfo> copy() const;

		private:
			Math::Real _radius, _height;
		};

		/**
		*	\class Mesh
		*	\brief Geometry mesh�� ������ �����ϴ� Ŭ����, GEOMETRY_TYPE = _MESH
		*/
		class Mesh :public GeometryInfo
		{
		public:
			/// �⺻ ������, �ּҴ� NULL�� �ʱ�ȭ �˴ϴ�,
			Mesh(const Math::SE3& T = (Math::SE3()), ///< Frame��ġ
				const Math::Vector4& Color = (Math::Vector4(-1, -1, -1, -1)) ///< (R, G, B, Alpha) ��
				) : GeometryInfo(GeometryInfo::_MESH, T, Color),
				_url("") {}
			/// Mesh ������ ����Ǿ� �ִ� �ּ� ���� �̿��Ͽ� �ʱ�ȭ�մϴ�.
			Mesh(const std::string& url, ///< Mesh ������ ����Ǿ� �ִ� �ּ�
				const Math::SE3& T = (Math::SE3()), ///< Frame��ġ
				const Math::Vector4& Color = (Math::Vector4(-1, -1, -1, -1)) ///< (R, G, B, Alpha) ��
				) : GeometryInfo(GeometryInfo::_MESH, T, Color),
				_url(url) {}

			/// Mesh ������ ����Ǿ� �ִ� �ּ� ���� �޾� �����մϴ�.
			void setUrl(const std::string& url ///< Mesh ������ ����Ǿ� �ִ� �ּ�
				)
			{
				_url = url;
			}

			/// ���� ���� �Ǿ��ִ� �ּ� ���� �˷��ݴϴ�.
			const std::string& getUrl() const
			{
				return _url;
			}

			/**
			*	\return �����ϸ� True, �������� ������ False
			*	\brief ���� ���� �Ǿ��ִ� �ּҿ� ������ �����ϴ��� �Ǵ�
			*/
			bool isValid() const;

			// ���� ����
			std::shared_ptr<GeometryInfo> copy() const;

		private:
			std::string _url;
		};

		/**
		*	\class UserModel
		*	\brief Geometry ������ ����ڰ� ������� ���� �� �ִ� Ŭ����, GEOMETRY_TYPE = _USERMODEL
		*	\todo addList ���ѷ��� ���� ���� �ʰ� �ϱ�, �˻�, ���� �Լ� �����
		*/
		class UserModel :public GeometryInfo
		{
		public:
			/// ������
			UserModel(const Math::SE3& T = (Math::SE3()), ///< Frame��ġ
				const Math::Vector4& Color = (Math::Vector4(-1, -1, -1, -1)) ///< (R, G, B, Alpha) ��
				) : GeometryInfo(GeometryInfo::_USERMODEL, T, Color), _GeometryList() {}

			/// srGeometry�ּҿ� SE3���� �޾Ƽ� �����մϴ�.
			void addList(const std::shared_ptr<GeometryInfo>& shape, const Math::SE3& T);

			/// List�κ��� ���� ã�ų� ������ �ٲٰ� ���� �� ����մϴ�.
			std::list< std::pair< std::shared_ptr<GeometryInfo>, Math::SE3 > >& getList()
			{
				return _GeometryList;
			}

			// ���� ����
			std::shared_ptr<GeometryInfo> copy() const;

		private:
			std::list< std::pair< std::shared_ptr<GeometryInfo>, Math::SE3 > > _GeometryList; ///< Geometry�� �����ϰ� �ִ� list ����
		};
	}
}