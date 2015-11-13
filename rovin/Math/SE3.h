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
			static SE3 Exp(const se3& S, const Real angle = (1.0));
			/// se3�� �� �κ� (w, v)���� �Է��� �޾� exponential mapping�� ���ݴϴ�.
			static SE3 Exp(const so3& w, const Vector3& v, const Real angle = (1.0));

			/// Log ���� ������ݴϴ�. ����� 6x1 se3�� �����ݴϴ�.
			static se3 Log(const SE3&);

			/// T�� Large Adjoint�� ���ݴϴ�.
			static Matrix6 Ad(const SE3& T);
			/// T^(-1)�� Large Adjoint�� ���ݴϴ�.
			static Matrix6 invAd(const SE3& T);

			/// Small Adjoint�� [S]�� ������ݴϴ�.
			static Matrix6 ad(const se3& S);

		private:
			SO3 _R;
			Vector3 _p;
		};

		/// [S]�� ������ݴϴ�.
		static Matrix4 Bracket(const se3& S)
		{
			so3 w = S.block(0, 0, 3, 1);
			Vector3 v = S.block(3, 0, 3, 1);

			Matrix4 result;
			result.setZero();
			result.block(0, 0, 3, 3) = Bracket(w);
			result.block(0, 3, 3, 1) = v;
			return result;
		}
	}
}