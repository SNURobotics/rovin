#include "SE3.h"

using namespace std;

namespace rovin
{
	namespace Math
	{
		Eigen::Matrix<Real, 1, 4> Affine_const(0, 0, 0, 1);

		SE3::SE3(const SE3& T)
		{
			_R._e = T._R._e;
			_p = T._p;
		}

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
			result._R._e.noalias() = _R._e * operand._R._e;
			result._p.noalias() = _R._e * operand._p + _p;
			return result;
		}

		SE3& SE3::operator *= (const SE3& operand)
		{
			_p = _R._e * operand._p + _p;
			_R._e = _R._e * operand._R._e;
			return *this;
		}

		const Matrix4 SE3::matrix() const
		{
			Matrix4 result;
			result << _R._e, _p, Affine_const;
			return result;
		}

		SE3 SE3::multiply(const SE3& op1, const SE3& op2, const SE3& op3)
		{
			SE3 result;
			Matrix3 tempMatrix;
			result._R._e.noalias() = (tempMatrix.noalias() = (op1._R._e * op2._R._e).eval()) * op3._R._e;
			result._p.noalias() = (tempMatrix * op3._p).eval() + (op1._R._e * op2._p).eval() + op1._p;
			return result;
		}

		SE3 SE3::multiply(const SE3& op1, const SE3& op2, const SE3& op3, const SE3& op4)
		{
			SE3 result;
			Matrix3 tempMatrix1, tempMatrix2;
			result._R._e.noalias() = (tempMatrix2.noalias() = ((tempMatrix1 = (op1._R._e * op2._R._e).eval()) * op3._R._e).eval()) * op4._R._e;
			result._p.noalias() = (tempMatrix2 * op4._p).eval() + (tempMatrix1 * op3._p).eval() + (op1._R._e * op2._p).eval() + op1._p;
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

		SE3 SE3::Exp(const se3& S, Real angle)
		{
			return Exp(S.head<3>(), S.tail<3>(), angle);
		}

		SE3 SE3::Exp(so3 w, Vector3 v, Real angle)
		{
			w *= angle;
			v *= angle;

			Real sq0 = w(0)*w(0), sq1 = w(1)*w(1), sq2 = w(2)*w(2);
			Real theta = sqrt(sq0 + sq1 + sq2);
			Real st_t, ct_t, vt_t;

			if (theta < RealEps)
			{
				st_t = 1.0 - theta*theta/6.0;
				ct_t = 0.5 - theta*theta/24.0;
				vt_t = (w(0)*v(0) + w(1)*v(1) + w(2)*v(2))*(1.0 - theta*theta/20.0)/6.0;
			}
			else
			{
				Real itheta = 1.0/theta;
				st_t = sin(theta)*itheta;
				itheta *= itheta;
				ct_t = (1.0 - cos(theta))*itheta;
				vt_t = (w(0)*v(0) + w(1)*v(1) + w(2)*v(2))*(1.0 - st_t)*itheta;
			}

			SE3 result;

			result._R._e(0, 0) = 1.0 - ct_t*(sq1 + sq2);
			result._R._e(0, 1) = ct_t * w(0) * w(1) - st_t * w(2);
			result._R._e(0, 2) = ct_t * w(0) * w(2) + st_t * w(1);
			result._R._e(1, 0) = ct_t * w(0) * w(1) + st_t * w(2);
			result._R._e(1, 1) = 1.0 - ct_t*(sq0 + sq2);
			result._R._e(1, 2) = ct_t * w(1) * w(2) - st_t * w(0);
			result._R._e(2, 0) = ct_t * w(0) * w(2) - st_t * w(1);
			result._R._e(2, 1) = ct_t * w(1) * w(2) + st_t * w(0);
			result._R._e(2, 2) = 1.0 - ct_t*(sq0 + sq1);
			result._p(0) = st_t * v(0) + vt_t * w(0) + ct_t * (w(1) * v(2) - w(2) * v(1));
			result._p(1) = st_t * v(1) + vt_t * w(1) + ct_t * (w(2) * v(0) - w(0) * v(2));
			result._p(2) = st_t * v(2) + vt_t * w(2) + ct_t * (w(0) * v(1) - w(1) * v(0));

			return result;
		}

		se3 SE3::Log(const SE3& T)
		{
			se3 result;

			so3 w = SO3::Log(T._R);

			if (w.norm() < RealEps)
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