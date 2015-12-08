/**
*	\file	SE3.h
*	\date	2015.11.05
*	\author	Keunjun (ckj@robotics.snu.ac.kr)
*	\brief	SE3 Ŭ����
*/

#pragma once

#include <iostream>
#include "Constant.h"
#include "SO3.h"

namespace rovin
{
	namespace Math
	{
		class SE3;
		typedef Vector6 se3;
		typedef Vector6 dse3;

		/**
		*	\class SE3
		*	\brief SE3�� �����ϰ� ó���ϴ� Ŭ����
		*/
		class SE3
		{
		public:
			///������
			SE3(const SE3& T);
			/// ȸ���� ���� �ʰ� p��ŭ �̵��� SE3�� �����մϴ�. 
			SE3(const Vector3& p = (Vector3::Zero())) : _R(), _p(p) {}
			/// R��ŭ ȸ���� �ϰ� p��ŭ �̵��� SE3�� �����մϴ�.
			SE3(const SO3& R, const Vector3& p = (Vector3::Zero())) : _R(R), _p(p) {}
			/// R��ŭ ȸ���� �ϰ� p��ŭ �̵��� SE3�� �����մϴ�. ��, R�� ������ 3x3 ����̹Ƿ� SO3�� �����ϴ��� Ȯ���� �մϴ�. SO3�� ������ �������� �ʴ� ��쿡�� ������ ���ϴ�.
			SE3(const Matrix3& R, const Vector3& p = (Vector3::Zero())) : _R(R), _p(p) {}
			/// ������ ����� T�� �޾Ƽ� T�� SE3�� �����ϸ� T�� �̿��Ͽ� SE3�� �����մϴ�.
			SE3(const Matrix4& T);

			/// SE3 �Ҹ���
			~SE3() {}

			SE3& operator = (const SE3&);
			SE3& operator = (const Matrix4&);
			SE3 operator * (const SE3&) const;
			SE3& operator *= (const SE3&);
			/**
			*	\warning SE3�� SE3���� ����ϴ� ���� ������.matrix�� ��ȯ�ϴ� �������� �ð��� ���� �ɸ�.
			*	\brief Matrixr4d�� ��ȯ���ݴϴ�.
			*/
			const Matrix4 matrix() const;

			/// ȸ���κ��� SO3�� R���� �����մϴ�.
			void setRotation(const SO3& R);
			/// ȸ���κ��� ������ ��� M���� �����մϴ�. �� ��쿡�� _M�� SO3�� ������ �����ϴ��� Ȯ���մϴ�.
			void setRotation(const Matrix3& M);
			/// Translation�κ��� p�� �����մϴ�.
			void setPosition(const Vector3& p);
			/// ȸ���κ��� SO3���·� �˷��ݴϴ�.
			const SO3& getRotation() const;
			/// Translation�κ��� Vector3d���·� �˷��ݴϴ�.
			const Vector3& getPosition() const;

			/// ��� �� �� ����մϴ�.
			friend std::ostream& operator << (std::ostream&, const SE3&);

			/// ������� �����ݴϴ�.
			SE3 inverse() const;

			/// se3�� S�� �̿��Ͽ� exponential mapping�� ���ݴϴ�.
			static SE3 Exp(const se3& S, Real angle = (1.0));
			/// se3�� �� �κ� (w, v)���� �Է��� �޾� exponential mapping�� ���ݴϴ�.
			static SE3 Exp(so3 w, Vector3 v, Real angle = (1.0));

			/// Log ���� ������ݴϴ�. ����� 6x1 se3�� �����ݴϴ�.
			static se3 Log(const SE3&);

			/// T�� Large Adjoint�� ���ݴϴ�.
			static Matrix6 Ad(const SE3& T);
			///
			static se3 Ad(const SE3& T, const se3& S);
			/// T^(-1)�� Large Adjoint�� ���ݴϴ�.
			static Matrix6 InvAd(const SE3& T);
			///
			static se3 InvAd(const SE3& T, const se3& S);

			/// Small Adjoint�� [S]�� ������ݴϴ�.
			static Matrix6 ad(const se3& S);
			/// Small Adjoint�� [S]^T�� ������ݴϴ�.
			static Matrix6 adTranspose(const se3& S);
			///
			static se3 ad(const se3& S1, const se3& S2);
			static se3 adTranspose(const se3& S1, const se3& S2);

		private:
			SO3 _R;
			Vector3 _p;
		};

		/// [S]�� ������ݴϴ�.
		static Matrix4 Bracket(const se3& S)
		{
			Matrix4 result;

			result(0, 0) = 0;
			result(0, 1) = -S(2);
			result(0, 2) = S(1);

			result(1, 0) = S(2);
			result(1, 1) = 0;
			result(1, 2) = -S(0);

			result(2, 0) = -S(1);
			result(2, 1) = S(0);
			result(2, 2) = 0;

			result(0, 3) = S(3);
			result(1, 3) = S(4);
			result(2, 3) = S(5);

			result(3, 0) = 0;
			result(3, 1) = 0;
			result(3, 2) = 0;
			result(3, 3) = 0;

			return result;
		}
	}
}