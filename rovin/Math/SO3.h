/**
*	\file	SO3.h
*	\date	2015.11.05
*	\author	Keunjun (ckj@robotics.snu.ac.kr)
*	\brief	SO3 클래스
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
		*	\brief SO3를 생성하고 처리하는 클래스
		*/
		class SO3
		{
			friend class SE3;

		public:
			/// R을 이용하여 생성합니다.
			SO3() : _e(Eigen::Matrix3d::Identity()) {}

			/// R과 같은 값을 갖도록 합니다. 
			SO3(const SO3& R) : _e(R._e) {}

			/// R과 같은 값을 갖도록 합니다. R=R^T와 det(R)=1을 만족하지 않는 경우에는 에러가 납니다.
			SO3(const Eigen::Matrix3d& R);

			/// SO3 소멸자
			~SO3() {}

			SO3& operator = (const SO3&);
			SO3& operator = (const Eigen::Matrix3d&);
			SO3 operator * (const SO3&) const;
			SO3& operator *= (const SO3&);
			/// Matrix3d로 변환해줍니다.
			const Eigen::Matrix3d& matrix() const;

			/// 1번째 열을 가지고 옵니다.
			Eigen::Vector3d getX() const;
			// 2번째 열을 가지고 옵니다.
			Eigen::Vector3d getY() const;
			/// 3번째 열을 가지고 옵니다.
			Eigen::Vector3d getZ() const;

			/// 출력 할 때 사용합니다.
			friend std::ostream& operator << (std::ostream&, const SO3&);

			/// 인버스 값을 돌려보냅니다.
			SO3 inverse() const;

			/// X축으로 angle만큼 회전합니다.
			static SO3 RotX(const double angle);
			/// Y축으로 angle만큼 회전합니다.
			static SO3 RotY(const double angle);
			/// Z축으로 angle만큼 회전합니다.
			static SO3 RotZ(const double angle);
			/// ZYX 오일러 행렬을 만듭니다. angle1, 2, 3는 순서대로 Z, Y, X에 해당하는 각도입니다.
			static SO3 EulerZYX(const double angle1, const double angle2, const double angle3);
			/// ZYZ 오일러 행렬을 만듭니다. angle1, 2, 3는 순서대로 Z, Y, Z에 해당하는 각도입니다.
			static SO3 EulerZYZ(const double angle1, const double angle2, const double angle3);

			/// R행렬을 ZYX 오일러 각으로 변환해줍니다. 리턴 값은 행렬 형태입니다.
			static Eigen::Vector3d invEulerZYX(const SO3& R);
			/// R행렬을 ZYZ 오일러 각으로 변환해줍니다. 리턴 값은 행렬 형태입니다.
			static Eigen::Vector3d invEulerZYZ(const SO3& R);

			/// Exponential 매핑을 해줍니다. so3인 w를 값으로 받습니다. (angle은 회전 각을 의미합니다. 결론적으로 w * angle로 exponential 매핑을 합니다.)
			static SO3 Exp(const so3& w, const double angle = (1.0));

			/// Log 값을 계산해줍니다. 결과는 3x1 so3로 돌려줍니다.
			static so3 Log(const SO3& R);

			/// 임의의 매트릭스를 SO3 manifold에 projection 시켜줍니다.
			static SO3 Projection(const Eigen::Matrix3d&);

		private:
			Eigen::Matrix3d _e;
		};

		/// [w]을 계산해줍니다.
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