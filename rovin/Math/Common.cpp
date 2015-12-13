#include "Common.h"

using namespace std;
using namespace Eigen;

namespace rovin
{
	namespace Math
	{
		MatrixX Function::Jacobian(const VectorX& x) const
		{
			int n = x.size();

			VectorX e(n);
			e.setZero();
			e(0) = 1.0;

			VectorX Fp = func(x + _eps*e);
			VectorX Fn = func(x - _eps*e);

			int m = Fp.size();

			MatrixX J(m, n);
			J.col(0) = (Fp - Fn) / (2 * _eps);

			for (int i = 1; i < n; i++)
			{
				e(i - 1) = 0.0;
				e(i) = 1.0;

				Fp = func(x + _eps*e);
				Fn = func(x - _eps*e);

				J.col(i) = (Fp - Fn) / (2 * _eps);
			}

			return J;
		}

		std::vector< MatrixX > Function::Hessian(const VectorX& x) const
		{
			Real eps_square = _eps*_eps;

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
				Fval.col(i) = func(x + _eps*ei);
				ei(i) = 0.0;
			}

			ei(0) = 1.0;
			ej(0) = 1.0;
			VectorX F1 = func(x + _eps*ei + _eps*ej);
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
				for (int j = 0; j <= i; j++)
				{
					ej(j) = 1.0;
					if (i == 0 && j == 0);
					else
					{
						F1 = func(x + _eps*ei + _eps*ej);

						Hij = (F1 - Fval.col(i) - Fval.col(j) + F) / eps_square;

						for (int k = 0; k < m; k++)
						{
							H[k](i, j) = Hij(k);
							H[k](j, i) = Hij(k);
						}
					}

					ej(j) = 0.0;
				}
				ei(i) = 0.0;
			}

			return H;
		}

		std::vector< MatrixX > Function::Hessian2(const VectorX& x) const
		{
			Real eps_square_inv = 1.0 / (_eps*_eps);

			int n = x.size();

			VectorX ei(n);
			VectorX ej(n);
			ei.setZero();
			ej.setZero();

			VectorX F = func(x);
			int m = F.size();

			MatrixX Fval1(m, n);
			MatrixX Fval2(m, n);
			MatrixX Hii(m, n);
			for (int i = 0; i < n; i++)
			{
				ei(i) = 1.0;
				Fval1.col(i) = func(x + _eps*ei);
				Fval2.col(i) = func(x - _eps*ei);
				ei(i) = 0.0;
			}

			vector< MatrixX > H;
			for (int i = 0; i < m; i++)
			{
				H.push_back(MatrixX(n, n));
				for (int j = 0; j < n; j++)
				{
					Hii(i, j) = (Fval1(i, j) + Fval2(i, j) - F(i) - F(i)) * eps_square_inv;
				}
			}

			VectorX F1(m);
			VectorX F2(m);
			VectorX Hij(m);

			for (int i = 0; i < n; i++)
			{
				ei(i) = 1.0;
				for (int j = 0; j < n; j++)
				{
					if (i == j)
					{
						for (int k = 0; k < m; k++)
						{
							H[k](i, i) = Hii(k, i);
						}
					}
					else if (i < j)
					{
						ej(j) = 1.0;

						F1 = func(x + _eps*ei + _eps*ej);
						F2 = func(x - _eps*ei - _eps*ej);
						Hij = 0.5 * ((F1 + F2 - F - F) * eps_square_inv - Hii.col(i) - Hii.col(j));

						for (int k = 0; k < m; k++)
						{
							H[k](i, j) = Hij(k);
						}

						ej(j) = 0.0;
					}
					else
					{
						for (int k = 0; k < m; k++)
						{
							H[k](i, j) = H[k](j, i);
						}
					}
				}
				ei(i) = 0.0;
			}

			return H;
		}

		std::vector< MatrixX > Function::Hessian4(const VectorX& x) const
		{
			Real eps_square_inv = 1.0 / (_eps*_eps);

			int n = x.size();

			VectorX ei(n);
			VectorX ej(n);
			ei.setZero();
			ej.setZero();

			VectorX F = func(x);
			int m = F.size();

			MatrixX Fval1(m, n);
			MatrixX Fval2(m, n);
			MatrixX Hii(m, n);
			for (int i = 0; i < n; i++)
			{
				ei(i) = 1.0;
				Fval1.col(i) = func(x + _eps*ei);
				Fval2.col(i) = func(x - _eps*ei);
				ei(i) = 0.0;
			}

			vector< MatrixX > H;
			for (int i = 0; i < m; i++)
			{
				H.push_back(MatrixX(n, n));
				for (int j = 0; j < n; j++)
				{
					Hii(i, j) = (Fval1(i, j) + Fval2(i, j) - F(i) - F(i)) * eps_square_inv;
				}
			}

			VectorX F1(m);
			VectorX F2(m);
			VectorX F3(m);
			VectorX F4(m);
			VectorX Hij(m);

			for (int i = 0; i < n; i++)
			{
				ei(i) = 1.0;
				for (int j = 0; j < n; j++)
				{
					if (i == j)
					{
						for (int k = 0; k < m; k++)
						{
							H[k](i, i) = Hii(k, i);
						}
					}
					else if (i < j)
					{
						ej(j) = 1.0;

						F1 = func(x + _eps*ei + _eps*ej);
						F2 = func(x - _eps*ei - _eps*ej);
						F3 = func(x - _eps*ei + _eps*ej);
						F4 = func(x + _eps*ei - _eps*ej);
						Hij = 0.25 * (F1 + F2 - F3 - F4) * eps_square_inv;

						for (int k = 0; k < m; k++)
						{
							H[k](i, j) = Hij(k);
						}

						ej(j) = 0.0;
					}
					else
					{
						for (int k = 0; k < m; k++)
						{
							H[k](i, j) = H[k](j, i);
						}
					}
				}
				ei(i) = 0.0;
			}

			return H;
		}

		VectorX LinearFunction::func(const Math::VectorX & x) const
		{
			return A*x + b;
		}

		MatrixX LinearFunction::Jacobian(const Math::VectorX & x) const
		{
			return A;
		}

		std::vector<MatrixX> LinearFunction::Hessian(const VectorX & x) const
		{
			vector<MatrixX> Hess(b.size());
			for (int i = 0; i < b.size(); i++)
				Hess[i] = MatrixX::Zero(x.size(), x.size());
			return Hess;
		}

		VectorX MultiObjectiveFunction::func(const VectorX & x) const
		{
			VectorX result;
			result = (*_functionList[0])(x);
			for (unsigned int i = 1; i < _functionList.size(); i++)
			{
				result += (*_functionList[i])(x);
			}
			return result;
		}

		MatrixX MultiObjectiveFunction::Jacobian(const VectorX & x) const
		{
			MatrixX result;
			result = (*_functionList[0]).Jacobian(x);
			for (unsigned int i = 1; i < _functionList.size(); i++)
			{
				result += (*_functionList[i]).Jacobian(x);
			}
			return result;
		}

		void MultiObjectiveFunction::addFunction(FunctionPtr func)
		{
			_functionList.push_back(func);
		}

		int findIdx(const VectorX& data, const Real& x)
		{
			int n = data.size();

			int start = 0;
			int end = n;
			int middle = (start + end) / 2;
			int result;

			while (start < end)
			{
				if (data(middle) <= x)
				{
					result = middle;
					start = middle + 1;
				}
				else
				{
					end = middle;
				}
				middle = (start + end) / 2;
			}

			if (result < 0) result = 0;
			if (result >= n) result = n - 1;

			return result;
		}

		VectorX BilinearInterpolation::operator ()(const Math::Real& x, const Math::Real& y)
		{
			int xidx = findIdx(_x, x);
			int yidx = findIdx(_y, y);

			Real x1 = _x(xidx);
			Real y1 = _y(yidx);
			Real x2;
			Real y2;

			if (xidx == _x.size() - 1 && yidx == _y.size() - 1)
				return _fs[xidx][yidx];			
			else if (xidx == _x.size() - 1)
			{
				y2 = _y(yidx + 1);
				return  ((y - y1) / (y2 - y1))*(_fs[xidx][yidx + 1] - _fs[xidx][yidx]) + _fs[xidx][yidx];
			}
			else if (yidx == _y.size() - 1)
			{
				x2 = _x(xidx + 1);
				return ((x - x1) / (x2 - x1))*(_fs[xidx + 1][yidx] - _fs[xidx][yidx]) + _fs[xidx][yidx];
			}
			else
			{
				x2 = _x(xidx + 1);
				y2 = _y(yidx + 1);
				return (1.0 / ((x2 - x1) * (y2 - y1))) * (_fs[xidx][yidx] * (x2 - x)*(y2 - y) + _fs[xidx + 1][yidx] * (x - x1)*(y2 - y) + _fs[xidx][yidx + 1] * (x2 - x)*(y - y1) + _fs[xidx + 1][yidx + 1] * (x - x1)*(y - y1));
			}
		}

		void BilinearInterpolation::setX(const VectorX& x)
		{
			_x = x;
		}

		void BilinearInterpolation::setY(const VectorX& y)
		{
			_y = y;
		}

		void BilinearInterpolation::setElements(const vector<vector<VectorX>>& fs)
		{
			_fs = fs;
		}
		VectorX LinearInterpolation::operator()(const Real & x)
		{
			int xidx = findIdx(_x, x);

			Real x1 = _x(xidx);
			if (xidx == _x.size() - 1)
				return _fs[xidx];
			else
			{
				Real x2 = _x(xidx + 1);
				return ((x - x1) / (x2 - x1))*(_fs[xidx + 1] - _fs[xidx]) + _fs[xidx];
			}
			
		}
		void LinearInterpolation::setX(const Math::VectorX & x)
		{
			_x = x;
		}
		void LinearInterpolation::setElements(const vector<VectorX>& fs)
		{
			_fs = fs;
		}
}
}