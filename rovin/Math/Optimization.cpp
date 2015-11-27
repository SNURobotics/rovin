#include "Optimization.h"

using namespace std;

namespace rovin
{
	namespace Math
	{
		MatrixX Function::Jacobian(const VectorX& x)
		{
			int n = x.size();

			VectorX e(n);
			e.setZero();
			e(0) = 1.0;

			VectorX Fp = func(x + eps*e);
			VectorX Fn = func(x - eps*e);

			int m = Fp.size();

			MatrixX J(m, n);
			J.col(0) = (Fp - Fn) / (2 * eps);

			for (int i = 1; i < n; i++)
			{
				e(i - 1) = 0.0;
				e(i) = 1.0;

				Fp = func(x + eps*e);
				Fn = func(x - eps*e);

				J.col(i) = (Fp - Fn) / (2 * eps);
			}

			return J;
		}

		std::vector< MatrixX > Function::Hessian(const VectorX& x)
		{
			Real eps_square = eps*eps;

			int n = x.size();

			VectorX ei(n);
			VectorX ej(n);
			ei.setZero();
			ej.setZero();

			VectorX F = func(x);
			int m = F.size();

			MatrixX Fval(m, n);
			for (int i = 0; i < n; i++)
			{
				ei(i) = 1.0;
				Fval.col(i) = func(x + eps*ei);
				ei(i) = 0.0;
			}

			ei(0) = 1.0;
			ej(0) = 1.0;
			VectorX F1 = func(x + eps*ei + eps*ej);
			ei(0) = 0.0;
			ej(0) = 0.0;

			VectorX Hij = (F1 - Fval.col(0) - Fval.col(0) + F) / eps_square;

			vector< MatrixX > H;
			for (int i = 0; i < m; i++)
			{
				H.push_back(MatrixX(n, n));
				H[i](0, 0) = Hij(i);
			}

			for (int i = 0; i < n; i++)
			{
				ei(i) = 1.0;
				for (int j = 0; j < n; j++)
				{
					ej(j) = 1.0;
					if (i == 0 && j == 0) continue;

					F1 = func(x + eps*ei + eps*ej);

					Hij = (F1 - Fval.col(i) - Fval.col(j) + F) / eps_square;

					for (int k = 0; k < m; k++)
					{
						H[k](i, j) = Hij(k);
					}

					ej(j) = 0.0;
				}
				ei(i) = 0.0;
			}

			return H;
		}
	}
}