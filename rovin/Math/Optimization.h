#pragma once

#include "Constant.h"
#include "Common.h"

#include <vector>
#include <memory>
#include <Eigen/Eigenvalues>
#include <nlopt.hpp>

namespace rovin
{
	namespace Math
	{
		class LineSearch
		{
		public:
			enum Algorithm
			{
				Backtracking
			};

			LineSearch(const Algorithm& algo = (Algorithm::Backtracking));

			Real solve(const VectorX& x, const VectorX& P);
			Real BacktrackingMethod(const VectorX& x, const VectorX& P);

			Algorithm _algorithm;

			FunctionPtr _objectiveFunc;

			Real _alpha;

			Real _alpha0;
			Real _tau;
			Real _c;
		};

		class QPOptimization
		{
		public:
			enum Algorithm
			{
				InteriorPoint
			};
			enum ExitFlag
			{
				SolutionFound,
				InfeasibleProblem,
				ExceedMaxIternation
			};

			QPOptimization(const Algorithm& algo = (Algorithm::InteriorPoint));
			QPOptimization(const MatrixX& G, const VectorX& g, const MatrixX& A = (MatrixX()), const VectorX& b = (VectorX()),
				const MatrixX& C = (MatrixX()), const VectorX& d = (VectorX()), const Algorithm& algo = (Algorithm::InteriorPoint));

			VectorX solve(const VectorX& x);
			VectorX InteriorPointMethod(const VectorX& x);

			Algorithm _algorithm;
			ExitFlag _exit;

			MatrixX _G;
			VectorX _g;
			MatrixX _A;
			VectorX _b;
			MatrixX _C;
			VectorX _d;

			VectorX _xf;
			VectorX _y;
			VectorX _z;
			VectorX _s;

			unsigned int _Iter;

			unsigned int _maxIteration;
			Real _stepSize;
			Real _tolCon;
			bool _Display;
		};

		class NonlinearOptimization
		{
		public:
			class MeritFunction : public Function
			{
			public:
				MeritFunction() {}

				VectorX func(const VectorX& x) const;

				FunctionPtr _f;
				FunctionPtr _ceq;
				FunctionPtr _cineq;

				Real _theta;
			};

			class ConstraintFunction : public Function
			{
			public:
				ConstraintFunction(int xN, int eqN, int ineqN) : _xN(xN), _eqN(eqN), _ineqN(ineqN) {}

				VectorX func(const VectorX& x) const;

				FunctionPtr _ceq;
				FunctionPtr _cineq;

				int _xN;
				int _eqN;
				int _ineqN;
			};

			enum Algorithm
			{
				SQP,
				NLopt
			};

			/////////////////////////////////////// NLOPT ////////////////////////////////////////////////////
			nlopt::opt opt;
			//Math::Real objective(const std::vector<double> &x, std::vector<double> &grad, void *data);
			//Math::Real eqconstraint(const std::vector<double> &x, std::vector<double> &grad, void *data);
			//Math::Real ineqconstraint(const std::vector<double> &x, std::vector<double> &grad, void *data);
			VectorX NLoptMethod(const VectorX& x);
			std::vector<double> obj_currentx;
			std::vector<double> eq_currentx;
			std::vector<double> ineq_currentx;
			Math::VectorX obj_fval;			Math::MatrixX obj_jacobian;
			Math::VectorX eq_fval;			Math::MatrixX eq_jacobian;
			Math::VectorX ineq_fval;		Math::MatrixX ineq_jacobian;
			bool obj, eq, ineq;
			//////////////////////////////////////////////////////////////////////////////////////////////////

			NonlinearOptimization(const Algorithm& algo = (Algorithm::NLopt));

			VectorX solve(const VectorX& x);
			VectorX SQPMethod(const VectorX& x);

			Algorithm _algorithm;

			VectorX _xf;

			unsigned int _Iter;

			FunctionPtr _objectiveFunc;
			FunctionPtr _eqFunc;
			FunctionPtr _ineqFunc;// < 0

			unsigned int _maxIteration;
			Real _tolCon;
			bool _Display;
		};

		class NewtonRapshon
		{
		public:
			NewtonRapshon()
			{
				_tolCon = 1e-8;
			}

			VectorX solve(const VectorX& x);

			VectorX _xf;

			FunctionPtr _func;
			Real _tolCon;
		};

		class ProjectToFeasibleSpace
		{
			class AugmentedFunction : public Function
			{
			public:
				AugmentedFunction(const int xN, const int inEqN);

				VectorX func(const VectorX& x) const;
				MatrixX Jacobian(const VectorX& x) const;

				int _xN;
				int _inEqN;

				FunctionPtr _eqConstraintFunc;
				FunctionPtr _inEqConstraintFunc;
			};
		public:
			ProjectToFeasibleSpace();

			VectorX project(const VectorX& x);

			FunctionPtr _eqConstraintFunc;
			FunctionPtr _inEqConstraintFunc;
			Real _tolCon;
			int _maxIter;
		};

		static const Real	OptEps = 1e-7;

		static MatrixX PDCorrection(const MatrixX& SquareMatrix)
		{
			int n = SquareMatrix.rows();
			Eigen::EigenSolver< MatrixX > es(SquareMatrix);
			MatrixX S(n, n);
			Real minValue = 0.0;
			S = es.eigenvalues().real().asDiagonal();
			for (int i = 0; i < n; i++)
			{
				if (RealLess(S(i, i), 0.0))
				{
					minValue = min(minValue, S(i, i));
				}
			}
			for (int i = 0; i < n; i++)
			{
				S(i, i) -= minValue - OptEps;
			}
			MatrixX V = es.eigenvectors().real();
			return V*S*V.inverse();
		}

		static bool OptRealEqual(const Real& operand1, const Real& operand2)
		{
			if (std::abs(operand1 - operand2) < OptEps + OptEps*std::abs(operand1))
			{
				return true;
			}
			return false;
		}

		static bool OptRealEqual(const VectorX& operand1, const Real& operand2)
		{
			unsigned int n = operand1.size();
			for (unsigned int i = 0; i < n; i++)
			{
				if (!OptRealEqual(operand1(i), operand2))
				{
					return false;
				}
			}
			return true;
		}

		static bool OptRealLessEqual(const Real& operand1, const Real& operand2)
		{
			if (operand1 < operand2 + OptEps + OptEps*std::abs(operand1))
			{
				return true;
			}
			return false;
		}

		static bool OptRealLessEqual(const VectorX& operand1, const Real& operand2)
		{
			unsigned int n = operand1.size();
			for (unsigned int i = 0; i < n; i++)
			{
				if (!OptRealLessEqual(operand1(i), operand2))
				{
					return false;
				}
			}
			return true;
		}

		static bool OptRealLess(const Real& operand1, const Real& operand2)
		{
			if (operand1 < operand2 - OptEps - OptEps*std::abs(operand1))
			{
				return true;
			}
			return false;
		}

		static bool OptRealBiggerEqual(const Real& operand1, const Real& operand2)
		{
			if (operand1 > operand2 - OptEps - OptEps*std::abs(operand1))
			{
				return true;
			}
			return false;
		}

		static bool OptRealBiggerEqual(const VectorX& operand1, const Real& operand2)
		{
			unsigned int n = operand1.size();
			for (unsigned int i = 0; i < n; i++)
			{
				if (!OptRealBiggerEqual(operand1(i), operand2))
				{
					return false;
				}
			}
			return true;
		}

		static bool OptRealBigger(const Real& operand1, const Real& operand2)
		{
			if (operand1 > operand2 + OptEps + OptEps*std::abs(operand1))
			{
				return true;
			}
			return false;
		}
	}
}