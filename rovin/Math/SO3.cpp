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
			return _e.block(0, 0, 3, 1);
		}

		Vector3 SO3::getY() const
		{
			return _e.block(0, 1, 3, 1);
		}

		Vector3 SO3::getZ() const
		{
			return _e.block(0, 2, 3, 1);
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
			result._e << 1, 0, 0,
				0, cos(angle), -sin(angle),
				0, sin(angle), cos(angle);
			return result;
		}

		SO3 SO3::RotY(const Real angle)
		{
			SO3 result;
			result._e << cos(angle), 0, sin(angle),
				0, 1, 0,
				-sin(angle), 0, cos(angle);
			return result;
		}

		SO3 SO3::RotZ(const Real angle)
		{
			SO3 result;
			result._e << cos(angle), -sin(angle), 0,
				sin(angle), cos(angle), 0,
				0, 0, 1;
			return result;
		}

		SO3 SO3::EulerZYX(const Real angle1, const Real angle2, const Real angle3)
		{
			SO3 result;
			Matrix3 R1, R2, R3;
			R1 << cos(angle1), -sin(angle1), 0,
				sin(angle1), cos(angle1), 0,
				0, 0, 1;
			R2 << cos(angle2), 0, sin(angle2),
				0, 1, 0,
				-sin(angle2), 0, cos(angle2);
			R3 << 1, 0, 0,
				0, cos(angle3), -sin(angle3),
				0, sin(angle3), cos(angle3);
			result._e = R1 * R2 * R3;
			return result;
		}

		SO3 SO3::EulerZYZ(const Real angle1, const Real angle2, const Real angle3)
		{
			SO3 result;
			Matrix3 R1, R2, R3;
			R1 << cos(angle1), -sin(angle1), 0,
				sin(angle1), cos(angle1), 0,
				0, 0, 1;
			R2 << cos(angle2), 0, sin(angle2),
				0, 1, 0,
				-sin(angle2), 0, cos(angle2);
			R3 << cos(angle3), -sin(angle3), 0,
				sin(angle3), cos(angle3), 0,
				0, 0, 1;
			result._e = R1 * R2 * R3;
			return result;
		}

		Vector3 SO3::invEulerZYX(const SO3& R)
		{
			return Vector3(atan2(R._e(1, 0), R._e(0, 0)), atan2(-R._e(2, 0), sqrt(R._e(0, 0)*R._e(0, 0) + R._e(1, 0)*R._e(1, 0))), atan2(R._e(2, 1), R._e(2, 2)));
		}

		Vector3 SO3::invEulerZYZ(const SO3& R)
		{
			return Vector3(atan2(R._e(1, 2), R._e(0, 2)), atan2(sqrt(R._e(0, 2)*R._e(0, 2) + R._e(1, 2)*R._e(1, 2)), R._e(2, 2)), atan2(R._e(2, 1), -R._e(2, 0)));
		}

		SO3 SO3::Exp(const so3& w, const Real angle)
		{
			SO3 result;

			if ((w * angle).norm() < Eigen::NumTraits<Real>::dummy_precision())
			{
				result._e = Matrix3::Identity();
			}
			else
			{
				so3 w_temp;
				Real angle_temp = angle;

				Real norm_w = w.norm();
				w_temp = w / norm_w;
				angle_temp = angle * norm_w;
				
				Matrix3 _bracket = Bracket(w_temp);

				result._e = Matrix3::Identity() + sin(angle_temp)*_bracket + (1 - cos(angle_temp))*_bracket*_bracket;
			}

			return result;
		}

		so3 SO3::Log(const SO3& R)
		{
			so3 result;

			if (R._e.isApprox(R._e.transpose()))
			{
				if (Matrix3::Identity().isApprox(R._e))
				{
					result << 0, 0, 0;
				}
				else
				{
					Matrix3 tmp1 = (R._e - Matrix3::Identity()) / 2;
					Real tmp2 = tmp1.trace() / 2;
					result << -tmp2 + tmp1(0, 0),
						-tmp2 + tmp1(1, 1),
						-tmp2 + tmp1(2, 2);
				}
			}
			else
			{
				Real theta = acos((R._e.trace() - 1) / 2);
				Matrix3 tmp = (R._e - R._e.transpose()) / (2 * sin(theta));
				result << tmp(2, 1),
					tmp(0, 2),
					tmp(1, 0);
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