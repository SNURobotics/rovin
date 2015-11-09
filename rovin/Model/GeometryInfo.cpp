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

		shared_ptr<GeometryInfo> Box::copy() const
		{
			return shared_ptr<GeometryInfo>(new Box(*this));
		}

		void Sphere::setRadius(const double& radius)
		{
			_radius = radius;
		}

		const double& Sphere::getRadius() const
		{
			return _radius;
		}

		shared_ptr<GeometryInfo> Sphere::copy() const
		{
			return shared_ptr<GeometryInfo>(new Sphere(*this));
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

		shared_ptr<GeometryInfo> Capsule::copy() const
		{
			return shared_ptr<GeometryInfo>(new Capsule(*this));
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

		shared_ptr<GeometryInfo> Cylinder::copy() const
		{
			return shared_ptr<GeometryInfo>(new Cylinder(*this));
		}

		bool Mesh::isValid() const
		{
			if (_access(_url.c_str(), 0) == 0)
			{
				return true;
			}
			return false;
		}

		shared_ptr<GeometryInfo> Mesh::copy() const
		{
			return shared_ptr<GeometryInfo>(new Mesh(*this));
		}

		void UserModel::addList(const std::shared_ptr<GeometryInfo>& shape, const Math::SE3& T)
		{
			_GeometryList.push_front(std::pair< std::shared_ptr<GeometryInfo>, Math::SE3 >(shape, T));
		}

		shared_ptr<GeometryInfo> UserModel::copy() const
		{
			UserModel *clone = new UserModel();
			clone->setType(this->getType());
			clone->setColor(this->getColor());
			clone->setFrame(this->getFrame());
			for (list< pair <shared_ptr<GeometryInfo>, Math::SE3> >::const_iterator iterPos = _GeometryList.begin(); iterPos != _GeometryList.end(); iterPos++)
			{
				clone->addList(iterPos->first->copy(), iterPos->second);
			}
			return shared_ptr<GeometryInfo>(clone);
		}
	}
}