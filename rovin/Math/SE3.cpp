#include "SE3.h"

using namespace std;

namespace rovin
{
	namespace Math
	{
		Eigen::Matrix<Real, 1, 4> Affine_const(0, 0, 0, 1);
		SE3::SE3(const Matrix4& T)
		{
			Eigen::Matrix<Real, 1, 4> Affine = T.block(3, 0, 1, 4);
			assert(Affine_const.isApprox(Affine));

			SE3::SE3(T.block(0, 0, 3, 3), T.block(0, 3, 1, 3));
		}

		SE3& SE3::operator = (const SE3& operand)
		{
			_R._e = operand._R._e;
			_p = operand._p;
			return *this;
		}

		SE3& SE3::operator = (const Matrix4& operand)
		{
			SE3::SE3(operand);
			return *this;
		}

		SE3 SE3::operator * (const SE3& operand) const
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

		const Matrix4 SE3::matrix() const
		{
			Matrix4 result;
			result << _R._e, _p, Affine_const;
			return result;
		}

		void SE3::setRotation(const SO3& R)
		{
			_R = R;
		}

		void SE3::setRotation(const Matrix3& M)
		{
			SO3 R(M);
			_R._e = R._e;
		}

		void SE3::setPosition(const Vector3& p)
		{
			_p = p;
		}

		const SO3& SE3::getRotation() const
		{
			return _R;
		}

		const Vector3& SE3::getPosition() const
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

		SE3 SE3::Exp(const se3& S, const Real angle)
		{
			so3 w = S.block(0, 0, 3, 1);
			Vector3 v = S.block(3, 0, 3, 1);
			return Exp(w, v, angle);
		}

		SE3 SE3::Exp(const so3& w, const Vector3& v, const Real angle)
		{
			SE3 result;

			if (Vector3::Zero().isApprox(w * angle))
			{
				result._R._e = Matrix3::Identity();
				result._p = v * angle;
			}
			else
			{
				so3 w_temp;
				Vector3 v_temp;
				Real angle_temp;

				Real norm_w = w.norm();
				w_temp = w / norm_w;
				v_temp = v / norm_w;
				angle_temp = angle * norm_w;

				Matrix3 _bracket = Bracket(w_temp);

				Matrix3 G;
				G = Matrix3::Identity()*angle_temp + (1 - cos(angle_temp))*_bracket + (angle_temp - sin(angle_temp))*_bracket*_bracket;

				result._R._e = SO3::Exp(w_temp, angle_temp)._e;
				result._p = G * v_temp;
			}

			return result;
		}

		se3 SE3::Log(const SE3& T)
		{
			se3 result;

			so3 w = SO3::Log(T._R);

			if (Vector3::Zero().isApprox(w))
			{
				result << w, T._p;
			}
			else
			{
				Real norm_w = w.norm();
				Matrix3 Ainv = Matrix3::Identity() - 0.5*Bracket(w) + (1 - (norm_w / 2) / tan(norm_w / 2))*Bracket(w)*Bracket(w) / (norm_w*norm_w);
				result << w, Ainv*T._p;
			}

			return result;
		}

		Matrix6 SE3::Ad(const SE3& T)
		{
			Matrix6 result;

			result << T._R._e, Matrix3::Zero(), Bracket(T._p)*T._R._e, T._R._e;

			return result;
		}

		Matrix6 SE3::invAd(const SE3& T)
		{
			Matrix6 result;
			Matrix3 InvR = T._R.inverse()._e;
			result << InvR, Matrix3::Zero(),
				-InvR*Bracket(T._p), InvR;

			return result;
		}

		Matrix6 SE3::ad(const se3& S)
		{
			Matrix6 result;

			so3 w = S.block(0, 0, 3, 1);
			Vector3 v = S.block(3, 0, 3, 1);

			result << Bracket(w), Matrix3::Zero(),
				Bracket(v), Bracket(w);

			return result;
		}
	}
}