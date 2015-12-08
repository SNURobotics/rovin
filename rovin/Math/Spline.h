#pragma once

#include "Constant.h"

#include <cassert>

namespace rovin
{
	namespace Math
	{
		template< int CoefficientN, int OrderK, int Dimension >
		class BSpline
		{
		public:
			BSpline()
			{

			}

			BSpline(const Math::VectorX& knots, const Eigen::Matrix<Real, Dimension, -1>& controlPoints)
			{
				_knots = knots;
				_controlPoints = controlPoints;

				assert(_knots.rows() - _controlPoints.cols() >= 0 && "BSpline�� ���� �� �����ϴ�.");

				_N = _controlPoints.cols();
				_K = _knots.rows() - _N;
				_D = _K - 1;
				_M = _controlPoints.rows();

				if((OrderK == -1) || (CoefficientN == -1)) _InvDeltaU = Eigen::Matrix<Real, -1, -1>(_K, _N + _K);
				_InvDeltaU.setZero();
				if(OrderK == -1) _d = Eigen::Matrix<Real, -1, -1>(_M, _K*_K);
				_d.setZero();

				int i;
				Real u0 = _knots(0);
				bool flag = false;
				for (i = 1; i < _N + _K; i++)
				{
					if (!RealEqual(u0, _knots(i))) flag = true;
					if (RealLessEqual(_knots(i - 1), _knots(i)));
					else break;
				}
				assert(i == _N + _K && "knots�� nondecreasing �Լ����� �մϴ�.");
				assert(flag && "The extreme knots should be different.");

				for (int j = 1; j <= _D; j++)
				{
					for (i = 0; i < _N + _D; i++)
					{
						if (i + _D + 1 - j < _N + _K && !RealEqual(_knots(i + _D + 1 - j), _knots(i)))
						{
							_InvDeltaU(j, i) = 1.0 / (_knots(i + _D + 1 - j) - _knots(i));
						}
					}
				}
			}

			~BSpline() {}

			Eigen::Matrix<Real, Dimension, 1> operator()(const Real& x)
			{
				return fval(x);
			}

			Eigen::Matrix<Real, Dimension, 1> fval(const Real& x)
			{
				if (RealLess(x, _knots(0)) || RealBiggerEqual(x, _knots(_N + _D)))
					return Math::VectorX::Zero(_M);

				int l = 0;

				int left, right, middle;
				left = 0;
				right = _N + _D - 1;
				middle = (left + right) / 2;
				while (left <= right)
				{
					if (RealLess(x, _knots(middle)))
					{
						right = middle - 1;
					}
					else
					{
						l = middle;
						left = middle + 1;
					}
					middle = (left + right) / 2;
				}

				int offset = l - _D;

				for (int i = 0; i <= _D; i++)
				{
					if (i + offset >= _N || i + offset < 0)
					{
						_d.col(0 * _K + i).setZero();
					}
					else
					{
						_d.col(0 * _K + i) = _controlPoints.col(i + offset);
					}
				}

				Real alpha;
				for (int j = 1; j <= _D; j++)
				{
					for (int i = offset + j; i <= l; i++)
					{
						if (i >= 0)
						{
							alpha = (x - _knots(i))*_InvDeltaU(j, i);
							_d.col(j * _K + i - offset) = (1 - alpha)*_d.col((j - 1) * _K + i - offset - 1) + alpha*_d.col((j - 1) * _K + i - offset);
						}
						else _d.col(j * _K + i - offset).setZero();
					}
				}

				return _d.col(_D * _K + _D);
			}

			BSpline<-1, -1, Dimension> derivative() const
			{
				int count = 0;
				for (int i = 0; i <= _N; i++)
				{
					if (RealEqual(_knots(i + _D), _knots(i)))
						count++;
				}

				Math::VectorX new_knots(_N + _K - count, 1);
				Math::MatrixX new_controlPoint(_M, _N + 1 - count);

				int cPntCount = 0;
				int knotCount = 0;
				if (!RealEqual(_knots(_D), _knots(0)))
				{
					new_controlPoint.col(cPntCount++) = _D * _controlPoints.col(0) / (_knots(_D) - _knots(0));
					new_knots(knotCount++) = _knots(0);
				}
				for (int i = 1; i < _N; i++)
				{
					if (!RealEqual(_knots(i + _D), _knots(i)))
					{
						new_controlPoint.col(cPntCount++) = _D * (_controlPoints.col(i) - _controlPoints.col(i - 1)) / (_knots(i + _D) - _knots(i));
						new_knots(knotCount++) = _knots(i);
					}
				}
				if (!RealEqual(_knots(_N + _D), _knots(_N)))
				{
					new_controlPoint.col(cPntCount++) = -_D * _controlPoints.col(_N - 1) / (_knots(_N + _D) - _knots(_N));
					new_knots(knotCount++) = _knots(_N);
				}
				for (int i = _N + 1; i <= _N + _D; i++)
				{
					new_knots(knotCount++) = _knots(i);
				}
				return BSpline<-1, -1, Dimension>(new_knots, new_controlPoint);
			}

		private:
			Eigen::Matrix<Real, ((OrderK == -1) || (CoefficientN == -1) ? -1 : CoefficientN + OrderK), 1> _knots;
			Eigen::Matrix<Real, Dimension, ((CoefficientN == -1) ? -1 : CoefficientN)> _controlPoints;

			Eigen::Matrix<Real, OrderK, ((OrderK == -1) || (CoefficientN == -1) ? -1 : CoefficientN + OrderK)> _InvDeltaU;
			Eigen::Matrix<Real, Dimension, (OrderK == -1 ? -1 : OrderK*OrderK)> _d;

			int _N;
			int _K;
			int _D;
			int _M;
		};
	}
}