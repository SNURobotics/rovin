#include "SE3.h"

using namespace std;
using namespace Eigen;

namespace rovin
{
	namespace Math
	{
		Matrix<double, 1, 4> Affine_const(0, 0, 0, 1);
		SE3::SE3(const Matrix4d& T)
		{
			Matrix<double, 1, 4> Affine = T.block(3, 0, 1, 4);
			assert(Affine_const.isApprox(Affine));

			SE3::SE3(T.block(0, 0, 3, 3), T.block(0, 3, 1, 3));
		}

		SE3& SE3::operator = (const SE3& operand)
		{
			_R._e = operand._R._e;
			_p = operand._p;
			return *this;
		}

		SE3& SE3::operator = (const Matrix4d& operand)
		{
			SE3::SE3(operand);
			return *this;
		}

		SE3 SE3::operator * (const SE3& operand)
		{
			SE3 result;
			result._R._e = _R._e * operand._R._e;
			result._p = _R._e * operand._p + _p;
			return result;
		}

		SE3& SE3::operator *= (const SE3& operand)
		{
			_R._e = _R._e * operand._R._e;
			_p = _R._e * operand._p + _p;
			return *this;
		}

		const Matrix4d SE3::matrix() const
		{
			Matrix4d result;
			result << _R._e, _p, Affine_const;
			return result;
		}

		void SE3::setRotation(const SO3& R)
		{
			_R = R;
		}

		void SE3::setRotation(const Matrix3d& M)
		{
			SO3 R(M);
			_R._e = R._e;
		}

		void SE3::setPosition(const Vector3d& p)
		{
			_p = p;
		}

		const SO3& SE3::getRotation() const
		{
			return _R;
		}

		const Vector3d& SE3::getPosition() const
		{
			return _p;
		}

		ostream& operator << (ostream& out, const SE3& T)
		{
			out << T.matrix();
			return out;
		}

		SE3 SE3::inverse() const
		{
			SE3 result;
			result._R._e = _R._e.transpose();
			result._p = -result._R._e * _p;
			return result;
		}

		SE3 SE3::Exp(const se3& S, const double angle)
		{
			so3 w = S.block(0, 0, 3, 1);
			Vector3d v = S.block(3, 0, 3, 1);
			return Exp(w, v, angle);
		}

		SE3 SE3::Exp(const so3& w, const Vector3d& v, const double angle)
		{
			SE3 result;

			if (Vector3d::Zero().isApprox(w * angle))
			{
				result._R._e = Matrix3d::Identity();
				result._p = v * angle;
			}
			else
			{
				so3 w_temp;
				Vector3d v_temp;
				double angle_temp;

				double norm_w = w.norm();
				w_temp = w / norm_w;
				v_temp = v / norm_w;
				angle_temp = angle * norm_w;

				Matrix3d G;
				G = Matrix3d::Identity()*angle_temp + (1 - cos(angle_temp))*Bracket(w_temp) + (angle_temp - sin(angle_temp))*Bracket(w_temp)*Bracket(w_temp);

				result._R._e = SO3::Exp(w_temp, angle_temp)._e;
				result._p = (Matrix3d)G * v_temp;
			}

			return result;
		}

		se3 SE3::Log(const SE3& T)
		{
			se3 result;

			so3 w = SO3::Log(T._R);

			if (Vector3d::Zero().isApprox(w))
			{
				result << w, T._p;
			}
			else
			{
				double norm_w = w.norm();
				Matrix3d Ainv = Matrix3d::Identity() - 0.5*Bracket(w) + (1 - (norm_w / 2) / tan(norm_w / 2))*Bracket(w)*Bracket(w) / (norm_w*norm_w);
				result << w, Ainv*T._p;
			}

			return result;
		}

		Matrix<double, 6, 6> SE3::Ad(const SE3& T)
		{
			Matrix<double, 6, 6> result;

			result << T._R._e, Matrix3d::Zero(), Bracket(T._p)*T._R._e, T._R._e;

			return result;
		}

		Matrix<double, 6, 6> SE3::invAd(const SE3& T)
		{
			Matrix<double, 6, 6> result;
			Matrix<double, 3, 3> InvR = T._R._e;

			result << InvR, Matrix3d::Zero(),
				-InvR*Bracket(T._p), InvR;

			return result;
		}

		Matrix<double, 6, 6> SE3::ad(const se3& S)
		{
			Matrix<double, 6, 6> result;

			so3 w = S.block(0, 0, 3, 1);
			Vector3d v = S.block(3, 0, 3, 1);

			result << Bracket(w), Matrix3d::Zero(),
				Bracket(v), Bracket(w);

			return result;
		}
	}
}