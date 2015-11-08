/**
*	\file	Link.cpp
*	\date	2015.11.05
*	\author	Keunjun (ckj@robotics.snu.ac.kr)
*	\brief	Link�� ������ �����ϴ� Ŭ���� ����
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
			assert(checkName(marker_name) && marker_name!=_name && "��Ŀ�� �̸����� ����� �� ���� �̸��� ���Խ��ϴ�.");

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
			assert(iter != _marker.end() && "�ش�Ǵ� �̸��� ���� marker�� ���� ���� �ʽ��ϴ�.");
			iter->second = T;
		}

		const Math::SE3& Link::getMarker(const string& marker_name) const
		{
			map< string, Math::SE3 >::const_iterator iter = _marker.find(marker_name);
			assert(iter != _marker.end() && "�ش�Ǵ� �̸��� ���� marker�� ���� ���� �ʽ��ϴ�.");
			return iter->second;
		}

		bool Link::isMarker(const string& marker_name) const
		{
			int count = _marker.count(marker_name);
			if (count == 1) return true;
			return false;
		}
	}
}