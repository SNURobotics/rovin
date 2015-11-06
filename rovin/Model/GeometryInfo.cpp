/**
*	\file	GeometryInfo.cpp
*	\date	2015.11.04
*	\author	Keunjun (ckj@robotics.snu.ac.kr)
*	\brief	GeometryInfo ±¸Çö
*/
#include "GeometryInfo.h"

#include <io.h>

using namespace std;
using namespace Eigen;

namespace rovin
{
	namespace Model
	{
		GeometryInfo::~GeometryInfo() {}

		void Box::setDimension(const double& width, const double& depth, const double& height)
		{
			_width = width;
			_depth = depth;
			_height = height;
		}

		void Box::setDimension(const Vector3d& config)
		{
			_width = config(0);
			_depth = config(1);
			_height = config(2);
		}

		void Box::setCube(const double& length)
		{
			_width = _depth = _height = length;
		}

		Vector3d Box::getDimension() const
		{
			return Vector3d(_width, _depth, _height);
		}

		void Sphere::setRadius(const double& radius)
		{
			_radius = radius;
		}

		const double& Sphere::getRadius() const
		{
			return _radius;
		}

		void Capsule::setDimension(const double& radius, const double& height)
		{
			_radius = radius;
			_height = height;
		}

		void Capsule::setDimension(const Vector2d& config)
		{
			_radius = config(0);
			_height = config(1);
		}

		Vector2d Capsule::getDimension() const
		{
			return Vector2d(_radius, _height);
		}

		void Cylinder::setDimension(const double& radius, const double& height)
		{
			_radius = radius;
			_height = height;
		}

		void Cylinder::setDimension(const Vector2d& config)
		{
			_radius = config(0);
			_height = config(1);
		}

		Vector2d Cylinder::getDimension() const
		{
			return Vector2d(_radius, _height);
		}

		bool Mesh::isValid() const
		{
			if (_access(_url.c_str(), 0) == 0)
			{
				return true;
			}
			return false;
		}

		void UserModel::addList(const std::shared_ptr<GeometryInfo>& shape, const Math::SE3& T)
		{
			_GeometryList.push_front(std::pair< std::shared_ptr<GeometryInfo>, Math::SE3 >(shape, T));
		}

		shared_ptr<GeometryInfo> copyGeometry(const shared_ptr<GeometryInfo>& geometry)
		{
			switch (geometry->getType())
			{
			case GeometryInfo::_BOX:
				return shared_ptr<GeometryInfo>(new Box(*static_pointer_cast<Box>(geometry)));
			case GeometryInfo::_SPHERE:
				return shared_ptr<GeometryInfo>(new Sphere(*static_pointer_cast<Sphere>(geometry)));
			case GeometryInfo::_CAPSULE:
				return shared_ptr<GeometryInfo>(new Capsule(*static_pointer_cast<Capsule>(geometry)));
			case GeometryInfo::_CYLINDER:
				return shared_ptr<GeometryInfo>(new Cylinder(*static_pointer_cast<Cylinder>(geometry)));
			case GeometryInfo::_MESH:
				return shared_ptr<GeometryInfo>(new Mesh(*static_pointer_cast<Mesh>(geometry)));
			case GeometryInfo::_USERMODEL:
				shared_ptr<GeometryInfo> tmpUser(new UserModel());
				list< pair <shared_ptr<GeometryInfo>, Math::SE3> >& glist = static_pointer_cast<UserModel>(geometry)->getList();
				for (list< pair <shared_ptr<GeometryInfo>, Math::SE3> >::iterator iterPos = glist.begin(); iterPos != glist.end(); iterPos++)
				{
					static_pointer_cast<UserModel>(tmpUser)->addList(copyGeometry(iterPos->first), iterPos->second);
				}
				return tmpUser;
				break;
			}
		}
	}
}