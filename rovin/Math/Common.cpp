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
		CubicSplineInterpolation::CubicSplineInterpolation(const Math::VectorX & x, const vector<Math::VectorX>& fs, const Math::VectorX & fp1, const Math::VectorX & fpn)
		{
			_x = x;
			_fs = fs;
			_df1 = fp1;
			_dfn = fpn;
			setddf();
			_isddfCalculated = true;

		}
		Math::VectorX CubicSplineInterpolation::operator()(const Math::Real & x)
		{
			if (!_isddfCalculated)
			{
				setddf();
				_isddfCalculated = true;
			}
			int xIdx = findIdx(_x, x);
			Real h, b, a;
			VectorX y(_dim);
			if (xIdx == _x.size() - 1)
			{
				y = _fs[xIdx];
			}
			else
			{
				h = _x(xIdx + 1) - _x(xIdx);
				if (RealEqual(h, 0.0))
					cout << "Bad input to routine spline." << endl;
				a = (_x(xIdx + 1) - x) / h;
				b = (x - _x(xIdx)) / h;
				y = a*_fs[xIdx] + b*_fs[xIdx + 1] + ((a*a*a - a)*_ddfs[xIdx] + (b*b*b - b)*_ddfs[xIdx + 1])*(h*h) / 6.0;
			}
			return y;
		}
		void CubicSplineInterpolation::setddf()
		{
			Real p, qn, sig, un;
			int n = _x.size();
			_dim = _fs[0].size();
			VectorX u(n - 1);
			_ddfs.resize(n);
			for (int j = 0; j < n; j++)
				_ddfs[j].resize(_dim);
			for (int i = 0; i < _dim; i++)
			{
				if (_df1(i) > 0.99e99)
				{
					// natural spline (second deriv = 0)
					_ddfs[0](i) = u(0) = 0.0;
				}
				else
				{
					// initial velocity constraint
					_ddfs[0](i) = -0.5;
					u(0) = (3.0 / (_x(1) - _x(0)))*((_fs[1](i) - _fs[0](i)) / (_x(1) - _x(0)) - _df1(i));
				}
				for (int j = 1; j < n - 1; j++)
				{
					sig = (_x(j) - _x(j - 1)) / (_x(j + 1) - _x(j - 1));
					p = sig*_ddfs[j - 1](i) + 2.0;
					_ddfs[j](i) = (sig - 1.0) / p;
					u(j) = (_fs[j + 1](i) - _fs[j](i)) / (_x(j + 1) - _x(j)) - (_fs[j](i) - _fs[j - 1](i)) / (_x(j) - _x(j - 1));
					u(j) = (6.0*u(j) / (_x(j + 1) - _x(j - 1)) - sig*u(j - 1)) / p;
				}
				if (_dfn(i) > 0.99e99)
				{
					// natural spline (second deriv = 0)
					qn = un = 0.0;
				}
				else
				{
					// final velocity constraint
					qn = 0.5;
					un = (3.0 / (_x(n - 1) - _x(n - 2)))*(_dfn(i) - (_fs[n - 1](i) - _fs[n - 2](i)) / (_x(n - 1) - _x(n - 2)));
				}
				_ddfs[n - 1](i) = (un - qn*u(n - 2)) / (qn*_ddfs[n - 2](i) + 1.0);
				for (int k = n - 2; k >= 0; k--)
					_ddfs[k](i) = _ddfs[k](i)*_ddfs[k + 1](i) + u(k);
			}

		}
		void CubicSplineInterpolation::setX(const Math::VectorX & x)
		{
			_x = x;
		}
		void CubicSplineInterpolation::setElements(const std::vector<Math::VectorX>& fs)
		{
			_fs = fs;
		}
		void CubicSplineInterpolation::setBoundaryConditions(const Math::VectorX & fp1, const Math::VectorX & fpn)
		{
			_df1 = fp1;
			_dfn = fpn;
		}

	}
}