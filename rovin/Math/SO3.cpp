#include <cassert>
#include <cmath>
#include "SO3.h"

using namespace std;

namespace rovin
{
	namespace Math
	{
		SO3::SO3(const Matrix3& R)
		{
			assert(Matrix3::Identity().isApprox(R*R.transpose()) && abs(R.determinant() - 1.0) <= Eigen::NumTraits<Real>::dummy_precision());
			_e = R;
		}

		SO3& SO3::operator = (const SO3& operand)
		{
			_e = operand._e;
			return *this;
		}

		SO3& SO3::operator = (const Matrix3& operand)
		{
			SO3::SO3(operand);
			return *this;
		}

		SO3 SO3::operator * (const SO3& operand) const
		{
			SO3 result;
			result._e = _e * operand._e;
			return result;
		}

		SO3& SO3::operator *= (const SO3& operand)
		{
			_e = _e * operand._e;
			return *this;
		}

		const Matrix3& SO3::matrix() const
		{
			return _e;
		}

		Vector3 SO3::getX() const
		{
			return Vector3(_e(0, 0), _e(1, 0), _e(2, 0));
		}

		Vector3 SO3::getY() const
		{
			return Vector3(_e(0, 1), _e(1, 1), _e(2, 1));
		}

		Vector3 SO3::getZ() const
		{
			return Vector3(_e(0, 2), _e(1, 2), _e(2, 2));
		}

		ostream& rovin::Math::operator << (ostream& out, const SO3& R)
		{
			out << R.matrix();
			return out;
		}

		SO3 SO3::inverse() const
		{
			SO3 result;
			result._e = _e.transpose();
			return result;
		}

		SO3 SO3::RotX(const Real angle)
		{
			SO3 result;

			result._e(0, 0) = 1;
			result._e(0, 1) = 0;
			result._e(0, 2) = 0;

			result._e(1, 0) = 0;
			result._e(1, 1) = cos(angle);
			result._e(1, 2) = -sin(angle);

			result._e(2, 0) = 0;
			result._e(2, 1) = sin(angle);
			result._e(2, 2) = cos(angle);

			return result;
		}

		SO3 SO3::RotY(const Real angle)
		{
			SO3 result;

			result._e(0, 0) = cos(angle);
			result._e(0, 1) = 0;
			result._e(0, 2) = sin(angle);

			result._e(1, 0) = 0;
			result._e(1, 1) = 1;
			result._e(1, 2) = 0;

			result._e(2, 0) = -sin(angle);
			result._e(2, 1) = 0;
			result._e(2, 2) = cos(angle);

			return result;
		}

		SO3 SO3::RotZ(const Real angle)
		{
			SO3 result;

			result._e(0, 0) = cos(angle);
			result._e(0, 1) = -sin(angle);
			result._e(0, 2) = 0;

			result._e(1, 0) = sin(angle);
			result._e(1, 1) = cos(angle);
			result._e(1, 2) = 0;

			result._e(2, 0) = 0;
			result._e(2, 1) = 0;
			result._e(2, 2) = 1;

			return result;
		}

		SO3 SO3::EulerZYX(const Real angle1, const Real angle2, const Real angle3)
		{
			return RotZ(angle1) * RotY(angle2) * RotX(angle3);
		}

		SO3 SO3::EulerZYZ(const Real angle1, const Real angle2, const Real angle3)
		{
			return RotZ(angle1) * RotY(angle2) * RotZ(angle3);
		}

		Vector3 SO3::invEulerZYX(const SO3& R)
		{
			return Vector3(atan2(R._e(1, 0), R._e(0, 0)), atan2(-R._e(2, 0), sqrt(R._e(0, 0)*R._e(0, 0) + R._e(1, 0)*R._e(1, 0))), atan2(R._e(2, 1), R._e(2, 2)));
		}

		Vector3 SO3::invEulerZYZ(const SO3& R)
		{
			return Vector3(atan2(R._e(1, 2), R._e(0, 2)), atan2(sqrt(R._e(0, 2)*R._e(0, 2) + R._e(1, 2)*R._e(1, 2)), R._e(2, 2)), atan2(R._e(2, 1), -R._e(2, 0)));
		}

		SO3 SO3::Exp(so3 w, Real angle)
		{
			w *= angle;

			Real sq0 = w(0)*w(0), sq1 = w(1)*w(1), sq2 = w(2)*w(2);
			Real theta = sqrt(sq0 + sq1 + sq2);
			Real st_t, ct_t;

			if (theta < RealEps)
			{
				st_t = 1.0 - theta*theta / 6.0;
				ct_t = 0.5 - theta*theta / 24.0;
			}
			else
			{
				Real itheta = 1.0 / theta;
				st_t = sin(theta)*itheta;
				itheta *= itheta;
				ct_t = (1.0 - cos(theta))*itheta;
			}

			SO3 result;

			result._e(0, 0) = 1.0 - ct_t*(sq1 + sq2);
			result._e(0, 1) = ct_t * w(0) * w(1) - st_t * w(2);
			result._e(0, 2) = ct_t * w(0) * w(2) + st_t * w(1);
			result._e(1, 0) = ct_t * w(0) * w(1) + st_t * w(2);
			result._e(1, 1) = 1.0 - ct_t*(sq0 + sq2);
			result._e(1, 2) = ct_t * w(1) * w(2) - st_t * w(0);
			result._e(2, 0) = ct_t * w(0) * w(2) - st_t * w(1);
			result._e(2, 1) = ct_t * w(1) * w(2) + st_t * w(0);
			result._e(2, 2) = 1.0 - ct_t*(sq0 + sq1);

			return result;
		}

		so3 SO3::Log(const SO3& R)
		{
			so3 result;

			if (R._e.isApprox(R._e.transpose()))
			{
				if (R._e.isApprox(Matrix3::Identity()))
				{
					result << 0, 0, 0;
				}
				else
				{
					Matrix3 tmp1 = (R._e - Matrix3::Identity()) / 2;
					Real tmp2 = tmp1.trace() / 2;
					result(0) = -tmp2 + tmp1(0, 0);
					result(1) = -tmp2 + tmp1(1, 1);
					result(2) = -tmp2 + tmp1(2, 2);
				}
			}
			else
			{
				Real theta = acos((R._e.trace() - 1) / 2);
				Matrix3 tmp = (R._e - R._e.transpose()) / (2 * sin(theta));
				result(0) = tmp(2, 1);
				result(1) = tmp(0, 2);
				result(2) = tmp(1, 0);
				result = result * theta;
			}

			return result;
		}

		SO3 SO3::Projection(const Matrix3& R)
		{
			SO3 result;

			Eigen::JacobiSVD<Matrix3> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);

			if (R.determinant() > 0)
			{
				result._e = svd.matrixV() * svd.matrixU().transpose();
			}
			else
			{
				result._e = svd.matrixV() * Eigen::DiagonalMatrix<Real, 3>(1, 1, -1) * svd.matrixU().transpose();
			}

			return result;
		}
	}
}