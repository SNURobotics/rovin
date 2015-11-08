/**
*	\file	SE3.h
*	\date	2015.11.05
*	\author	Keunjun (ckj@robotics.snu.ac.kr)
*	\brief	SE3 Ŭ����
*/

#pragma once

#include <iostream>
#include <Eigen/Dense>
#include "SO3.h"

namespace rovin
{
	namespace Math
	{
		class SE3;
		typedef Eigen::Matrix<double, 6, 1> se3;
		typedef Eigen::Matrix<double, 6, 1> dse3;

		/**
		*	\class SE3
		*	\brief SE3�� �����ϰ� ó���ϴ� Ŭ����
		*/
		class SE3
		{
		public:
			/// ȸ���� ���� �ʰ� p��ŭ �̵��� SE3�� �����մϴ�. 
			SE3(const Eigen::Vector3d& p = (Eigen::Vector3d::Zero())) : _R(), _p(p) {}
			/// R��ŭ ȸ���� �ϰ� p��ŭ �̵��� SE3�� �����մϴ�.
			SE3(const SO3& R, const Eigen::Vector3d& p = (Eigen::Vector3d::Zero())) : _R(R), _p(p) {}
			/// R��ŭ ȸ���� �ϰ� p��ŭ �̵��� SE3�� �����մϴ�. ��, R�� ������ 3x3 ����̹Ƿ� SO3�� �����ϴ��� Ȯ���� �մϴ�. SO3�� ������ �������� �ʴ� ��쿡�� ������ ���ϴ�.
			SE3(const Eigen::Matrix3d& R, const Eigen::Vector3d& p = (Eigen::Vector3d::Zero())) : _R(R), _p(p) {}
			/// ������ ����� T�� �޾Ƽ� T�� SE3�� �����ϸ� T�� �̿��Ͽ� SE3�� �����մϴ�.
			SE3(const Eigen::Matrix4d& T);

			/// SE3 �Ҹ���
			~SE3() {}

			SE3& operator = (const SE3&);
			SE3& operator = (const Eigen::Matrix4d&);
			SE3 operator * (const SE3&) const;
			SE3& operator *= (const SE3&);
			/**
			*	\warning SE3�� SE3���� ����ϴ� ���� ������.matrix�� ��ȯ�ϴ� �������� �ð��� ���� �ɸ�.
			*	\brief Matrixr4d�� ��ȯ���ݴϴ�.
			*/
			const Eigen::Matrix4d matrix() const;

			/// ȸ���κ��� SO3�� R���� �����մϴ�.
			void setRotation(const SO3& R);
			/// ȸ���κ��� ������ ��� M���� �����մϴ�. �� ��쿡�� _M�� SO3�� ������ �����ϴ��� Ȯ���մϴ�.
			void setRotation(const Eigen::Matrix3d& M);
			/// Translation�κ��� p�� �����մϴ�.
			void setPosition(const Eigen::Vector3d& p);
			/// ȸ���κ��� SO3���·� �˷��ݴϴ�.
			const SO3& getRotation() const;
			/// Translation�κ��� Vector3d���·� �˷��ݴϴ�.
			const Eigen::Vector3d& getPosition() const;

			/// ��� �� �� ����մϴ�.
			friend std::ostream& operator << (std::ostream&, const SE3&);

			/// ������� �����ݴϴ�.
			SE3 inverse() const;

			/// se3�� S�� �̿��Ͽ� exponential mapping�� ���ݴϴ�.
			static SE3 Exp(const se3& S, const double angle = (1.0));
			/// se3�� �� �κ� (w, v)���� �Է��� �޾� exponential mapping�� ���ݴϴ�.
			static SE3 Exp(const so3& w, const Eigen::Vector3d& v, const double angle = (1.0));

			/// Log ���� ������ݴϴ�. ����� 6x1 se3�� �����ݴϴ�.
			static se3 Log(const SE3&);

			/// T�� Large Adjoint�� ���ݴϴ�.
			static Eigen::Matrix<double, 6, 6> Ad(const SE3& T);
			/// T^(-1)�� Large Adjoint�� ���ݴϴ�.
			static Eigen::Matrix<double, 6, 6> invAd(const SE3& T);

			/// Small Adjoint�� [S]�� ������ݴϴ�.
			static Eigen::Matrix<double, 6, 6> ad(const se3& S);

		private:
			SO3 _R;
			Eigen::Vector3d _p;
		};

		/// [S]�� ������ݴϴ�.
		static Eigen::Matrix4d Bracket(const se3& S)
		{
			so3 w = S.block(0, 0, 3, 1);
			Eigen::Vector3d v = S.block(3, 0, 3, 1);

			Eigen::Matrix4d result;
			result.setZero();
			result.block(0, 0, 3, 3) = Bracket(w);
			result.block(0, 3, 3, 1) = v;
			return result;
		}
	}
}