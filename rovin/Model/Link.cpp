/**
*	\file	Link.cpp
*	\date	2015.11.05
*	\author	Keunjun (ckj@robotics.snu.ac.kr)
*	\brief	Link의 정보를 저장하는 클래스 정의
*/

#include "Link.h"

#include <cassert>

using namespace std;
using namespace Eigen;
using namespace rovin::Model;

namespace rovin
{
	namespace Model
	{
		void Link::addMarker(const string& marker_name, const Math::SE3& T, const bool overwrite)
		{
			if (overwrite)
			{
				_marker[marker_name] = T;
			}
			else
			{
				_marker.insert(pair< string, Math::SE3 >(marker_name, T));
			}
		}

		void Link::deleteMarker(const string& marker_name)
		{
			_marker.erase(marker_name);
		}

		void Link::changeMarker(const string& marker_name, const Math::SE3& T)
		{
			map< string, Math::SE3 >::iterator iter = _marker.find(marker_name);
			assert(iter != _marker.end());
			iter->second = T;
		}

		const Math::SE3& Link::findMarker(const string& marker_name) const
		{
			map< string, Math::SE3 >::const_iterator iter = _marker.find(marker_name);
			assert(iter != _marker.end());
			return iter->second;
		}

		bool Link::checkMarker(const string& marker_name) const
		{
			int count = _marker.count(marker_name);
			if (count == 1) return true;
			return false;
		}

		void Link::addJoint(const string& joint_name, const weak_ptr<Joint>& joint_pointer, const Math::SE3& T)
		{
			_joint.push_front(make_tuple(joint_name, joint_pointer, T));
		}

		void Link::deleteJoint(const std::string& joint_name)
		{
			for (list< tuple< string, weak_ptr<Joint>, Math::SE3 > >::const_iterator iter = _joint.begin(); iter != _joint.end(); iter++)
			{
				if (get<0>(*iter) == joint_name)
				{
					_joint.erase(iter);
				}
			}
		}
	}
}