/**
*	\file	SO3.h
*	\date	2015.11.05
*	\author	Keunjun (ckj@robotics.snu.ac.kr)
*	\brief	SO3 Ŭ����
*/

#pragma once

#include <iostream>
#include <Eigen/Dense>

namespace rovin
{
	namespace Math
	{
		class SO3;
		typedef Eigen::Vector3d so3;
		typedef Eigen::Vector3d dso3;

		/**
		*	\class SO3
		*	\brief SO3�� �����ϰ� ó���ϴ� Ŭ����
		*/
		class SO3
		{
			friend class SE3;

		public:
			/// R�� �̿��Ͽ� �����մϴ�.
			SO3() : _e(Eigen::Matrix3d::Identity()) {}

			/// R�� ���� ���� ������ �մϴ�. 
			SO3(const SO3& R) : _e(R._e) {}

			/// R�� ���� ���� ������ �մϴ�. R=R^T�� det(R)=1�� �������� �ʴ� ��쿡�� ������ ���ϴ�.
			SO3(const Eigen::Matrix3d& R);

			/// SO3 �Ҹ���
			~SO3() {}

			SO3& operator = (const SO3&);
			SO3& operator = (const Eigen::Matrix3d&);
			SO3 operator * (const SO3&) const;
			SO3& operator *= (const SO3&);
			/// Matrix3d�� ��ȯ���ݴϴ�.
			const Eigen::Matrix3d& matrix() const;

			/// 1��° ���� ������ �ɴϴ�.
			Eigen::Vector3d getX() const;
			// 2��° ���� ������ �ɴϴ�.
			Eigen::Vector3d getY() const;
			/// 3��° ���� ������ �ɴϴ�.
			Eigen::Vector3d getZ() const;

			/// ��� �� �� ����մϴ�.
			friend std::ostream& operator << (std::ostream&, const SO3&);

			/// �ι��� ���� ���������ϴ�.
			SO3 inverse() const;

			/// X������ angle��ŭ ȸ���մϴ�.
			static SO3 RotX(const double angle);
			/// Y������ angle��ŭ ȸ���մϴ�.
			static SO3 RotY(const double angle);
			/// Z������ angle��ŭ ȸ���մϴ�.
			static SO3 RotZ(const double angle);
			/// ZYX ���Ϸ� ����� ����ϴ�. angle1, 2, 3�� ������� Z, Y, X�� �ش��ϴ� �����Դϴ�.
			static SO3 EulerZYX(const double angle1, const double angle2, const double angle3);
			/// ZYZ ���Ϸ� ����� ����ϴ�. angle1, 2, 3�� ������� Z, Y, Z�� �ش��ϴ� �����Դϴ�.
			static SO3 EulerZYZ(const double angle1, const double angle2, const double angle3);

			/// R����� ZYX ���Ϸ� ������ ��ȯ���ݴϴ�. ���� ���� ��� �����Դϴ�.
			static Eigen::Vector3d invEulerZYX(const SO3& R);
			/// R����� ZYZ ���Ϸ� ������ ��ȯ���ݴϴ�. ���� ���� ��� �����Դϴ�.
			static Eigen::Vector3d invEulerZYZ(const SO3& R);

			/// Exponential ������ ���ݴϴ�. so3�� w�� ������ �޽��ϴ�. (angle�� ȸ�� ���� �ǹ��մϴ�. ��������� w * angle�� exponential ������ �մϴ�.)
			static SO3 Exp(const so3& w, const double angle = (1.0));

			/// Log ���� ������ݴϴ�. ����� 3x1 so3�� �����ݴϴ�.
			static so3 Log(const SO3& R);

			/// ������ ��Ʈ������ SO3 manifold�� projection �����ݴϴ�.
			static SO3 Projection(const Eigen::Matrix3d&);

		private:
			Eigen::Matrix3d _e;
		};

		/// [w]�� ������ݴϴ�.
		static Eigen::Matrix3d Bracket(const so3 w)
		{
			Eigen::Matrix3d result;
			result << 0, -w(2), w(1),
				w(2), 0, -w(0),
				-w(1), w(0), 0;
			return result;
		}
	}
}