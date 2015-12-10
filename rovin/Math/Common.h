#pragma once

#include "Constant.h"

#include <memory>
#include <vector>
#include <cmath>

namespace rovin
{
	namespace Math
	{
		class Function;
		typedef std::shared_ptr< Function > FunctionPtr;

		class Function
		{
		public:
			Function(const Real& eps = (1e-5))
			{
				_eps = eps;
			}

			VectorX operator()(const VectorX& x) const
			{
				return func(x);
			}

			virtual VectorX func(const VectorX& x) const = 0;
			virtual MatrixX Jacobian(const VectorX& x) const;
			virtual std::vector< MatrixX > Hessian(const VectorX& x) const;
			virtual std::vector< MatrixX > Hessian2(const VectorX& x) const;
			virtual std::vector< MatrixX > Hessian4(const VectorX& x) const;

		private:
			Real _eps;
		};

		class MultiObjectiveFunction : public Function
		{
		public:
			MultiObjectiveFunction() : _functionList(std::vector<FunctionPtr>()) {}

			VectorX func(const VectorX& x) const;
			MatrixX Jacobian(const VectorX& x) const;

			void addFunction(FunctionPtr func);

			std::vector<FunctionPtr> _functionList;
		};

		class EmptyFunction : public Function
		{
		public:
			EmptyFunction() {}

			VectorX func(const VectorX& x) const { return VectorX::Ones(1)*(-1); }
			MatrixX Jacobian(const VectorX& x) const { return MatrixX::Zero(1, x.size()); }
			std::vector< MatrixX > Hessian(const VectorX& x) const { std::vector< MatrixX > hs(1); hs[0] = MatrixX::Zero(x.size(), x.size()); return hs; }
		};

		class LinearFunction : public Math::Function
		{
			
		public:
			LinearFunction() {}
			Math::MatrixX A;
			Math::VectorX b;

			Math::VectorX func(const Math::VectorX& x) const;
			Math::MatrixX Jacobian(const Math::VectorX& x) const;
			std::vector< Math::MatrixX > Hessian(const Math::VectorX& x) const;
		};

		class BilinearInterpolation
		{
		public:
			BilinearInterpolation() {}

			Math::VectorX operator ()(const Math::Real& x, const Math::Real& y);

			void setX(const Math::VectorX& x);
			void setY(const Math::VectorX& y);
			
			void setElements(const std::vector<std::vector<Math::VectorX>>& fs);

			Math::VectorX _x;
			Math::VectorX _y;
			std::vector<std::vector<Math::VectorX>> _fs;
		};

		static Real min(Real x, Real y)
		{
			if (x < y)
			{
				return x;
			}
			return y;
		}

		static VectorX min(const VectorX& x, Real y)
		{
			VectorX result = x;
			for (int i = 0; i < result.size(); i++)
			{
				result(i) = min(result(i), y);
			}
			return result;
		}

		static Real max(Real x, Real y)
		{
			if (x < y)
			{
				return y;
			}
			return x;
		}

		static VectorX max(const VectorX& x, Real y)
		{
			VectorX result = x;
			for (int i = 0; i < result.size(); i++)
			{
				result(i) = max(result(i), y);
			}
			return result;
		}

		static void fsincos(Real theta, Real& sine, Real& cosine)
		{
			theta -= (int)(theta*Inv_PI_DOUBLE)*PI_DOUBLE;
			if (theta < 0) theta += PI_DOUBLE;

			sine = sin(theta);
			if (theta < PI_HALF)
			{
				cosine = sqrt(1 - sine*sine);
				return;
			}
			else if (theta < PI + PI_HALF)
			{
				cosine = -sqrt(1 - sine*sine);
				return;
			}
			cosine = sqrt(1 - sine*sine);
		}

		static MatrixX pInv(const MatrixX& mat)
		{
			Eigen::JacobiSVD<Math::MatrixX> svd(mat, Eigen::ComputeFullU | Eigen::ComputeFullV);
			Real tolerance = RealEps;
			VectorX singular_values = svd.singularValues();
			MatrixX S(mat.rows(), mat.cols());
			S.setZero();
			for (int i = 0; i < singular_values.size(); i++)
			{
				if (singular_values(i) > tolerance)
				{
					S(i, i) = 1.0 / singular_values(i);
				}
				else
				{
					S(i, i) = 0;
				}
			}
			return svd.matrixV() * S.transpose() * svd.matrixU().transpose();
		}
	}
}