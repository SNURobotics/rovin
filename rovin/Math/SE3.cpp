#include "SE3.h"

#include "Numeric.h"

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
			result._R._e = _R._e * operand._R._e;
			result._p = _R._e * operand._p + _p;
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

			result.block<3, 3>(0, 0) = _R._e;
			result.block<3, 1>(0, 3) = _p;
			result.block<1, 4>(3, 0) = Affine_const;

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
			result._R._e = _R._e.transpose().eval();
			result._p = (-result._R._e) * _p;
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
				Real s, c;
				Math::fsincos_precise(theta, s, c);
				Real itheta = 1.0/theta;
				st_t = s*itheta;
				itheta *= itheta;
				ct_t = (1.0 - c)*itheta;
				vt_t = (w(0)*v(0) + w(1)*v(1) + w(2)*v(2))*(1.0 - st_t)*itheta;
			}

			SE3 result;

			Real *R = &result._R._e(0);
			Real *p = &result._p(0);
			*R = 1.0 - ct_t*(sq1 + sq2);
			*(R + 3) = ct_t * w(0) * w(1) - st_t * w(2);
			*(R + 6) = ct_t * w(0) * w(2) + st_t * w(1);
			*(R + 1) = ct_t * w(0) * w(1) + st_t * w(2);
			*(R + 4) = 1.0 - ct_t*(sq0 + sq2);
			*(R + 7) = ct_t * w(1) * w(2) - st_t * w(0);
			*(R + 2) = ct_t * w(0) * w(2) - st_t * w(1);
			*(R + 5) = ct_t * w(1) * w(2) + st_t * w(0);
			*(R + 8) = 1.0 - ct_t*(sq0 + sq1);
			*p = st_t * v(0) + vt_t * w(0) + ct_t * (w(1) * v(2) - w(2) * v(1));
			*(p + 1) = st_t * v(1) + vt_t * w(1) + ct_t * (w(2) * v(0) - w(0) * v(2));
			*(p + 2) = st_t * v(2) + vt_t * w(2) + ct_t * (w(0) * v(1) - w(1) * v(0));

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
				Matrix3 bw = Bracket(w);
				Matrix3 Ainv = Matrix3::Identity() - 0.5*bw.eval() + (1 - (norm_w / 2) / tan(norm_w / 2))*(bw*bw).eval() / (norm_w*norm_w);
				result << w, Ainv*T._p;
			}

			return result;
		}

		Matrix6 SE3::Ad(const SE3& T)
		{
			Matrix6 result;

			result(0, 0) = T._R._e(0, 0);
			result(0, 1) = T._R._e(0, 1);
			result(0, 2) = T._R._e(0, 2);

			result(0, 3) = 0;
			result(0, 4) = 0;
			result(0, 5) = 0;

			result(1, 0) = T._R._e(1, 0);
			result(1, 1) = T._R._e(1, 1);
			result(1, 2) = T._R._e(1, 2);

			result(1, 3) = 0;
			result(1, 4) = 0;
			result(1, 5) = 0;

			result(2, 0) = T._R._e(2, 0);
			result(2, 1) = T._R._e(2, 1);
			result(2, 2) = T._R._e(2, 2);

			result(2, 3) = 0;
			result(2, 4) = 0;
			result(2, 5) = 0;

			result(3, 0) = -T._p(2)*T._R._e(1, 0) + T._p(1)*T._R._e(2, 0);
			result(3, 1) = -T._p(2)*T._R._e(1, 1) + T._p(1)*T._R._e(2, 1);
			result(3, 2) = -T._p(2)*T._R._e(1, 2) + T._p(1)*T._R._e(2, 2);

			result(3, 3) = T._R._e(0, 0);
			result(3, 4) = T._R._e(0, 1);
			result(3, 5) = T._R._e(0, 2);

			result(4, 0) = T._p(2)*T._R._e(0, 0) - T._p(0)*T._R._e(2, 0);
			result(4, 1) = T._p(2)*T._R._e(0, 1) - T._p(0)*T._R._e(2, 1);
			result(4, 2) = T._p(2)*T._R._e(0, 2) - T._p(0)*T._R._e(2, 2);

			result(4, 3) = T._R._e(1, 0);
			result(4, 4) = T._R._e(1, 1);
			result(4, 5) = T._R._e(1, 2);

			result(5, 0) = -T._p(1)*T._R._e(0, 0) + T._p(0)*T._R._e(1, 0);
			result(5, 1) = -T._p(1)*T._R._e(0, 1) + T._p(0)*T._R._e(1, 1);
			result(5, 2) = -T._p(1)*T._R._e(0, 2) + T._p(0)*T._R._e(1, 2);

			result(5, 3) = T._R._e(2, 0);
			result(5, 4) = T._R._e(2, 1);
			result(5, 5) = T._R._e(2, 2);

			return result;
		}

		Matrix6 SE3::InvAd(const SE3& T)
		{
			Matrix6 result;

			result(0, 0) = T._R._e(0, 0);
			result(0, 1) = T._R._e(1, 0);
			result(0, 2) = T._R._e(2, 0);

			result(0, 3) = 0;
			result(0, 4) = 0;
			result(0, 5) = 0;

			result(1, 0) = T._R._e(0, 1);
			result(1, 1) = T._R._e(1, 1);
			result(1, 2) = T._R._e(2, 1);

			result(1, 3) = 0;
			result(1, 4) = 0;
			result(1, 5) = 0;

			result(2, 0) = T._R._e(0, 2);
			result(2, 1) = T._R._e(1, 2);
			result(2, 2) = T._R._e(2, 2);

			result(2, 3) = 0;
			result(2, 4) = 0;
			result(2, 5) = 0;

			result(3, 0) = -T._R._e(1, 0)*T._p(2) + T._R._e(2, 0)*T._p(1);
			result(3, 1) = T._R._e(0, 0)*T._p(2) - T._R._e(2, 0)*T._p(0);
			result(3, 2) = -T._R._e(0, 0)*T._p(1) + T._R._e(1, 0)*T._p(0);

			result(3, 3) = T._R._e(0, 0);
			result(3, 4) = T._R._e(1, 0);
			result(3, 5) = T._R._e(2, 0);

			result(4, 0) = -T._R._e(1, 1)*T._p(2) + T._R._e(2, 1)*T._p(1);
			result(4, 1) = T._R._e(0, 1)*T._p(2) - T._R._e(2, 1)*T._p(0);
			result(4, 2) = -T._R._e(0, 1)*T._p(1) + T._R._e(1, 1)*T._p(0);

			result(4, 3) = T._R._e(0, 1);
			result(4, 4) = T._R._e(1, 1);
			result(4, 5) = T._R._e(2, 1);

			result(5, 0) = -T._R._e(1, 2)*T._p(2) + T._R._e(2, 2)*T._p(1);
			result(5, 1) = T._R._e(0, 2)*T._p(2) - T._R._e(2, 2)*T._p(0);
			result(5, 2) = -T._R._e(0, 2)*T._p(1) + T._R._e(1, 2)*T._p(0);

			result(5, 3) = T._R._e(0, 2);
			result(5, 4) = T._R._e(1, 2);
			result(5, 5) = T._R._e(2, 2);

			return result;
		}

		Matrix6 SE3::ad(const se3& S)
		{
			Matrix6 result;

			so3 w = S.block(0, 0, 3, 1);
			Vector3 v = S.block(3, 0, 3, 1);

			result.block<3, 3>(0, 0) = Bracket(w);
			result.block<3, 3>(0, 3).setZero();
			result.block<3, 3>(3, 0) = Bracket(v);
			result.block<3, 3>(3, 3) = Bracket(w);

			return result;
		}
	}
}