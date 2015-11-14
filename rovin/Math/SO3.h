/**
*	\file	SO3.h
*	\date	2015.11.05
*	\author	Keunjun (ckj@robotics.snu.ac.kr)
*	\brief	SO3 Ŭ����
*/

#pragma once

#include <iostream>
#include "Constant.h"

namespace rovin
{
	namespace Math
	{
		class SO3;
		typedef Vector3 so3;
		typedef Vector3 dso3;

		/**
		*	\class SO3
		*	\brief SO3�� �����ϰ� ó���ϴ� Ŭ����
		*/
		class SO3
		{
			friend class SE3;

		public:
			/// R�� �̿��Ͽ� �����մϴ�.
			SO3() : _e(Matrix3::Identity()) {}

			/// R�� ���� ���� ������ �մϴ�. 
			SO3(const SO3& R) : _e(R._e) {}

			/// R�� ���� ���� ������ �մϴ�. R=R^T�� det(R)=1�� �������� �ʴ� ��쿡�� ������ ���ϴ�.
			SO3(const Matrix3& R);

			/// SO3 �Ҹ���
			~SO3() {}

			SO3& operator = (const SO3&);
			SO3& operator = (const Matrix3&);
			SO3 operator * (const SO3&) const;
			SO3& operator *= (const SO3&);
			/// Matrix3d�� ��ȯ���ݴϴ�.
			const Matrix3& matrix() const;

			/// 1��° ���� ������ �ɴϴ�.
			Vector3 getX() const;
			// 2��° ���� ������ �ɴϴ�.
			Vector3 getY() const;
			/// 3��° ���� ������ �ɴϴ�.
			Vector3 getZ() const;

			/// ��� �� �� ����մϴ�.
			friend std::ostream& operator << (std::ostream&, const SO3&);

			/// �ι��� ���� ���������ϴ�.
			SO3 inverse() const;

			/// X������ angle��ŭ ȸ���մϴ�.
			static SO3 RotX(const Real angle);
			/// Y������ angle��ŭ ȸ���մϴ�.
			static SO3 RotY(const Real angle);
			/// Z������ angle��ŭ ȸ���մϴ�.
			static SO3 RotZ(const Real angle);
			/// ZYX ���Ϸ� ����� ����ϴ�. angle1, 2, 3�� ������� Z, Y, X�� �ش��ϴ� �����Դϴ�.
			static SO3 EulerZYX(const Real angle1, const Real angle2, const Real angle3);
			/// ZYZ ���Ϸ� ����� ����ϴ�. angle1, 2, 3�� ������� Z, Y, Z�� �ش��ϴ� �����Դϴ�.
			static SO3 EulerZYZ(const Real angle1, const Real angle2, const Real angle3);

			/// R����� ZYX ���Ϸ� ������ ��ȯ���ݴϴ�. ���� ���� ��� �����Դϴ�.
			static Vector3 invEulerZYX(const SO3& R);
			/// R����� ZYZ ���Ϸ� ������ ��ȯ���ݴϴ�. ���� ���� ��� �����Դϴ�.
			static Vector3 invEulerZYZ(const SO3& R);

			/// Exponential ������ ���ݴϴ�. so3�� w�� ������ �޽��ϴ�. (angle�� ȸ�� ���� �ǹ��մϴ�. ��������� w * angle�� exponential ������ �մϴ�.)
			static SO3 Exp(so3 w, Real angle = (1.0));

			/// Log ���� ������ݴϴ�. ����� 3x1 so3�� �����ݴϴ�.
			static so3 Log(const SO3& R);

			/// ������ ��Ʈ������ SO3 manifold�� projection �����ݴϴ�.
			static SO3 Projection(const Matrix3&);

		private:
			Matrix3 _e;
		};

		/// [w]�� ������ݴϴ�.
		static Matrix3 Bracket(const so3 w)
		{
			Matrix3 result;
			result << 0, -w(2), w(1),
				w(2), 0, -w(0),
				-w(1), w(0), 0;
			return result;
		}
	}
}