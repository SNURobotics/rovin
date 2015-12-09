#include "Optimization.h"

#include <iostream>

using namespace std;
using namespace Eigen;

namespace rovin
{
	namespace Math
	{
		LineSearch::LineSearch(const Algorithm& algo)
		{
			if (algo == LineSearch::Algorithm::Backtracking)
			{
				_algorithm = LineSearch::Algorithm::Backtracking;
				_alpha0 = 1.0;
				_tau = 0.90;
				_c = 0.1;
			}
		}

		Real LineSearch::solve(const VectorX& x, const VectorX& P)
		{
			if (_algorithm == LineSearch::Algorithm::Backtracking)
			{
				return BacktrackingMethod(x, P);
			}
			return _tau;
		}

		Real LineSearch::BacktrackingMethod(const VectorX& x, const VectorX& P)
		{
			_alpha = _alpha0;
			Real t = -_c*((*_objectiveFunc).Jacobian(x)*P)(0);
			Real fval = (*_objectiveFunc)(x)(0);
			while (RealLess(fval - (*_objectiveFunc)(x + _alpha*P)(0), _alpha*t) && _alpha >= RealEps)
			{
				_alpha *= _tau;
			}
			return _alpha;
		}

		QPOptimization::QPOptimization(const QPOptimization::Algorithm& algo) : QPOptimization(MatrixX(), VectorX(), MatrixX(), VectorX(), MatrixX(), VectorX(), algo) {}

		QPOptimization::QPOptimization(const MatrixX& G, const VectorX& g, const MatrixX& A, const VectorX& b, const MatrixX& C, const VectorX& d, const Algorithm& algo) :
			_G(G), _g(g), _A(A), _b(b), _C(C), _d(d), _Display(false)
		{
			if (algo == QPOptimization::Algorithm::InteriorPoint)
			{
				_algorithm = QPOptimization::Algorithm::InteriorPoint;
				_maxIteration = 300;
				_stepSize = 0.95;
				_tolCon = 1e-6;
			}
		}

		VectorX QPOptimization::solve(const VectorX& x)
		{
			if (_algorithm == QPOptimization::InteriorPoint)
			{
				return InteriorPointMethod(x);
			}
			return x;
		}

		VectorX QPOptimization::InteriorPointMethod(const VectorX& x)
		{
			_xf = x;

			int xN = _xf.size();
			if (_G.cols() == xN && _G.rows() == xN && _g.size() == xN);
			else return _xf;

			if (_A.rows() == 0)
			{
				_A = MatrixX::Zero(1, xN);
				_b = VectorX::Zero(1);
			}
			int eqN = _A.rows();
			if (_A.cols() == xN && _b.size() == eqN);
			else return _xf;
			if (!OptRealEqual(_A*(_A.transpose()*_A).ldlt().solve(_A.transpose()*_b) - _b, 0.0))
			{
				_exit = ExitFlag::InfeasibleProblem;
				return _xf;
			}

			if (_C.rows() == 0)
			{
				_C = MatrixX::Zero(1, xN);
				_d = VectorX::Zero(1);
			}
			int ineqN = _C.rows();
			if (_C.cols() == xN && _d.size() == ineqN);
			else return _xf;

			_y = VectorX::Ones(eqN);
			_z = VectorX::Ones(ineqN);
			_s = VectorX::Ones(ineqN);
			VectorX e = VectorX::Ones(ineqN);

			MatrixX S, Z, invS, invZ, SAff, ZAff;
			S = _s.asDiagonal();
			Z = _z.asDiagonal();
			invS = S;
			invZ = Z;

			VectorX residualL, residualA, residualC, residualSZ;
			residualL = _G*_xf + _g - _A.transpose()*_y + _C.transpose()*_z;
			residualA = _A*_xf - _b;
			residualC = _s + _C*_xf - _d;
			residualSZ = S*Z*e;

			Real mu, muAff, sigma;
			mu = (_s.transpose()*_z)(0) / (Real)ineqN;

			MatrixX Gbar(xN + eqN, xN + eqN);
			Real alpha;
			VectorX delta, deltaX, deltaY, deltaZ, deltaS;
			Real alphaAff;
			VectorX deltaAff, deltaXAff, deltaYAff, deltaZAff, deltaSAff;

			Eigen::LDLT< MatrixX > GbarLDL;
			VectorX extendedResidual(xN + eqN);

			Gbar.setZero();
			Gbar.block(0, xN, xN, eqN) = -_A.transpose();
			Gbar.block(xN, 0, eqN, xN) = _A;

			for (_Iter = 0; _Iter < _maxIteration; _Iter++)
			{
				int i;
				for (i = 0; i < ineqN; i++)
				{
					invS(i, i) = 1.0 / S(i, i);
				}
				for (i = 0; i < ineqN; i++)
				{
					invZ(i, i) = 1.0 / Z(i, i);
				}
				Gbar.block(0, 0, xN, xN) = _G + _C.transpose()*invS*Z*_C;
				GbarLDL = Gbar.ldlt();

				extendedResidual.block(0, 0, xN, 1) = -residualL - _C.transpose()*(invS*Z)*(residualC - invZ*residualSZ);
				extendedResidual.block(xN, 0, eqN, 1) = -residualA;

				deltaAff = GbarLDL.solve(extendedResidual);
				deltaXAff = deltaAff.block(0, 0, xN, 1);
				deltaYAff = deltaAff.block(0, 0, eqN, 1);
				deltaZAff = invS*Z*_C*deltaXAff + invS*Z*(residualC - invZ*residualSZ);
				deltaSAff = -invZ*(residualSZ + S*deltaZAff);

				alphaAff = 1.0;
				for (i = 0; i < ineqN; i++)
				{
					if (RealLess(deltaSAff(i), 0.0))
					{
						alphaAff = min(alphaAff, -_s(i) / deltaSAff(i));
					}
				}
				for (i = 0; i < ineqN; i++)
				{
					if (RealLess(deltaZAff(i), 0.0))
					{
						alphaAff = min(alphaAff, -_z(i) / deltaZAff(i));
					}
				}

				muAff = ((_s + alphaAff*deltaSAff).transpose()*(_z + alphaAff*deltaZAff))(0) / (Real)ineqN;
				sigma = muAff / mu;
				sigma = sigma * sigma * sigma;

				SAff = deltaSAff.asDiagonal();
				ZAff = deltaZAff.asDiagonal();
				residualSZ += SAff*ZAff*e - sigma*mu*e;

				extendedResidual.block(0, 0, xN, 1) = -residualL - _C.transpose()*(invS*Z)*(residualC - invZ*residualSZ);
				extendedResidual.block(xN, 0, eqN, 1) = -residualA;

				delta = GbarLDL.solve(extendedResidual);
				deltaX = delta.block(0, 0, xN, 1);
				deltaY = delta.block(0, 0, eqN, 1);
				deltaZ = invS*Z*_C*deltaX + invS*Z*(residualC - invZ*residualSZ);
				deltaS = -invZ*(residualSZ + S*deltaZ);

				alpha = 1.0;
				for (i = 0; i < ineqN; i++)
				{
					if (RealLess(deltaS(i), 0.0))
					{
						alpha = min(alpha, -_s(i) / deltaS(i));
					}
				}
				for (i = 0; i < ineqN; i++)
				{
					if (RealLess(deltaZ(i), 0.0))
					{
						alpha = min(alpha, -_z(i) / deltaZ(i));
					}
				}

				_xf += _stepSize*alpha*deltaX;
				_y += _stepSize*alpha*deltaY;
				_s += _stepSize*alpha*deltaS;
				_z += _stepSize*alpha*deltaZ;

				S = _s.asDiagonal();
				Z = _z.asDiagonal();

				residualL = _G*_xf + _g - _A.transpose()*_y + _C.transpose()*_z;
				residualA = _A*_xf - _b;
				residualC = _s + _C*_xf - _d;
				residualSZ = S*Z*e;

				mu = (_s.transpose()*_z)(0) / (Real)ineqN;

				if (OptRealLessEqual(abs(mu), _tolCon))
				{
					_exit = ExitFlag::SolutionFound;
					break;
				}
			}
			if (!(OptRealEqual(_A*_xf - _b, 0.0) && OptRealLessEqual(_C*_xf - _d, 0.0)) || !OptRealLessEqual(_xf, 1 / OptEps))
			{
				_exit = ExitFlag::InfeasibleProblem;
			}
			else if (_Iter == _maxIteration)
			{
				_exit = ExitFlag::ExceedMaxIternation;
			}
			return _xf;
		}

		VectorX NonlinearOptimization::MeritFunction::func(const VectorX& x) const
		{
			VectorX result(1);
			result(0) = (*_f)(x)(0) + _theta*((*_ceq)(x).cwiseAbs().sum() + max((*_cineq)(x), 0.0).sum());
			return result;
		}

		VectorX NonlinearOptimization::ConstraintFunction::func(const VectorX& x) const
		{
			VectorX result(_eqN + _ineqN);
			result.head(_eqN) = (*_ceq)(x.head(_xN));
			result.tail(_ineqN) = (*_cineq)(x.head(_xN)) + x.tail(_ineqN);
			return result;
		}

		NonlinearOptimization::NonlinearOptimization(const Algorithm& algo) : _Display(false)
		{
			if (algo == NonlinearOptimization::Algorithm::SQP)
			{
				_algorithm = NonlinearOptimization::Algorithm::SQP;
				_tolCon = 1e-5;
				_maxIteration = 5000;
			}
			else if (algo == NonlinearOptimization::Algorithm::NLopt)
			{
				_algorithm = NonlinearOptimization::Algorithm::NLopt;
				obj = eq = ineq = false;
			}
		}

		VectorX NonlinearOptimization::solve(const VectorX& x)
		{
			if (_algorithm == NonlinearOptimization::SQP)
			{
				return SQPMethod(x);
			}
			else if (_algorithm == NonlinearOptimization::Algorithm::NLopt)
			{
				return NLoptMethod(x);
			}
			return x;
		}

		double objective(unsigned n, const double* x, double* grad, void *f_data)
		{
			NonlinearOptimization* ptr = reinterpret_cast<NonlinearOptimization*>(f_data);
			VectorX xv(n);
			for (int i = 0; i < n; i++) xv(i) = x[i];
			if (grad)
			{
				MatrixX jacobian = (*ptr->_objectiveFunc).Jacobian(xv);
				for (int j = 0; j < n; j++)
				{
					grad[j] = jacobian(0, j);
				}
			}
			return (*ptr->_objectiveFunc)(xv)(0);
		}

		void mineqconstraint(unsigned m, double* result, unsigned n, const double* x, double* grad, void* f_data)
		{
			NonlinearOptimization* ptr = reinterpret_cast<NonlinearOptimization*>(f_data);
			VectorX xv(n);
			for (int i = 0; i < n; i++) xv(i) = x[i];
			if (grad)
			{
				MatrixX jacobian = (*ptr->_ineqFunc).Jacobian(xv);
				for (int i = 0; i < m; i++)
				{
					for (int j = 0; j < n; j++)
					{
						grad[i*n + j] = jacobian(i, j);
					}
				}
			}
			VectorX fval = (*ptr->_ineqFunc)(xv);
			for (int i = 0; i < m; i++)
			{
				result[i] = fval(i);
			}
		}

		void meqconstraint(unsigned m, double* result, unsigned n, const double* x, double* grad, void* f_data)
		{
			NonlinearOptimization* ptr = reinterpret_cast<NonlinearOptimization*>(f_data);
			VectorX xv(n);
			for (int i = 0; i < n; i++) xv(i) = x[i];
			if (grad)
			{
				MatrixX jacobian = (*ptr->_eqFunc).Jacobian(xv);
				for (int i = 0; i < m; i++)
				{
					for (int j = 0; j < n; j++)
					{
						grad[i*n + j] = jacobian(i, j);
					}
				}
			}
			VectorX fval = (*ptr->_eqFunc)(xv);
			for (int i = 0; i < m; i++)
			{
				result[i] = fval(i);
			}
		}

		VectorX NonlinearOptimization::NLoptMethod(const VectorX& x)
		{
			_xf = x;

			int xN = _xf.size();
			int eqN = (*_eqFunc)(_xf).size();
			int ineqN = (*_ineqFunc)(_xf).size();

			opt = nlopt::opt(nlopt::LD_MMA, xN);
			opt.set_min_objective(objective, this);
			opt.add_inequality_mconstraint(mineqconstraint, this, vector<Real>(ineqN, 1e-8));
			//opt.add_equality_mconstraint(meqconstraint, this, vector<Real>(eqN, 1e-8));
			opt.set_xtol_rel(1e-4);
			opt.set_ftol_rel(1e-4);

			std::vector<Real> xi(xN);
			for (int i = 0; i < xN; i++)
			{
				xi[i] = _xf(i);
			}
			double minf;
			nlopt::result result = opt.optimize(xi, minf);
			for (int i = 0; i < xN; i++)
			{
				_xf(i) = xi[i];
			}
			return _xf;
		}

		VectorX NonlinearOptimization::SQPMethod(const VectorX& x)
		{
			_xf = x;

			int xN = _xf.size();
			int eqN = (*_eqFunc)(_xf).size();
			int ineqN = (*_ineqFunc)(_xf).size();

			VectorX lambda = VectorX::Zero(eqN);
			VectorX mu = VectorX::Zero(ineqN);

			// direction
			VectorX d(xN);

			// InitialGuess
			//ProjectToFeasibleSpace projectToFeasibleSpace;
			//projectToFeasibleSpace._eqConstraintFunc = _eqFunc;
			//projectToFeasibleSpace._inEqConstraintFunc = _ineqFunc;
			//_xf = projectToFeasibleSpace.project(_xf);

			// QPOptimization
			QPOptimization qpSolver;
			qpSolver._G = PDCorrection((*_objectiveFunc).Hessian(_xf)[0]);
			qpSolver._g = (*_objectiveFunc).Jacobian(_xf).transpose();
			qpSolver._A = (*_eqFunc).Jacobian(_xf);
			qpSolver._b = -(*_eqFunc)(_xf);
			qpSolver._C = (*_ineqFunc).Jacobian(_xf);
			qpSolver._d = -(*_ineqFunc)(_xf);
			qpSolver._y = VectorX::Zero(eqN);
			qpSolver._z = VectorX::Zero(ineqN);
			qpSolver._exit = QPOptimization::ExitFlag::SolutionFound;

			// Merit Function
			FunctionPtr meritFunction = FunctionPtr(new MeritFunction());
			static_pointer_cast<MeritFunction>(meritFunction)->_f = _objectiveFunc;
			static_pointer_cast<MeritFunction>(meritFunction)->_ceq = _eqFunc;
			static_pointer_cast<MeritFunction>(meritFunction)->_cineq = _ineqFunc;
			static_pointer_cast<MeritFunction>(meritFunction)->_theta = 0.0;

			// LineSearch
			LineSearch linesearchSolver;
			linesearchSolver._objectiveFunc = meritFunction;
			//linesearchSolver._alpha0 = 1.0;

			Real alpha, theta, fval, fval_last;
			MatrixX H(xN, xN);
			std::vector< MatrixX > Hessian;
			VectorX y, z;

			d = VectorX::Zero(xN);
			fval = (*_objectiveFunc)(_xf)(0);
			// DISPLAY

			for (_Iter = 0; _Iter < _maxIteration; _Iter++)
			{
				if (qpSolver._exit != QPOptimization::ExitFlag::InfeasibleProblem)
				{
					y = qpSolver._y;
					z = qpSolver._z;
				}
				d = qpSolver.solve(d);
				if (qpSolver._exit == QPOptimization::ExitFlag::InfeasibleProblem)
				{
					d = qpSolver._G.ldlt().solve(-qpSolver._g).normalized();
				}
				else
				{
					theta = RealMin;
					for (int i = 0; i < eqN; i++)
					{
						theta = max(std::abs(qpSolver._y(i)), theta);
					}
					for (int i = 0; i < ineqN; i++)
					{
						theta = max(qpSolver._z(i), theta);
					}
					if (RealBiggerEqual(static_pointer_cast<MeritFunction>(meritFunction)->_theta, theta));
					else static_pointer_cast<MeritFunction>(meritFunction)->_theta = max(static_pointer_cast<MeritFunction>(meritFunction)->_theta * 2, theta);
				}
				alpha = linesearchSolver.solve(_xf, d);
				_xf += alpha*d;

				// DISPLAY
				fval_last = fval;
				fval = (*_objectiveFunc)(_xf)(0);
				cout << fval << " " << alpha << endl;
				//cout << d << endl;
				//cout << (*_objectiveFunc).Jacobian(_xf) << endl;

				if (OptRealLessEqual(alpha*d.norm(), _tolCon) || OptRealEqual(fval_last, fval))
				{
					break;
				}

				H = (*_objectiveFunc).Hessian(_xf)[0];
				if (qpSolver._exit != QPOptimization::ExitFlag::InfeasibleProblem)
				{
					qpSolver._y = y + alpha*(qpSolver._y - y);
					qpSolver._z = z + alpha*(qpSolver._z - z);

					Hessian = (*_eqFunc).Hessian(_xf);
					for (int i = 0; i < eqN; i++)
					{
						H += qpSolver._y(i) * Hessian[i];
					}
					Hessian = (*_ineqFunc).Hessian(_xf);
					for (int i = 0; i < ineqN; i++)
					{
						H += qpSolver._z(i) * Hessian[i];
					}
				}
				qpSolver._G = PDCorrection(H);
				qpSolver._g = (*_objectiveFunc).Jacobian(_xf).transpose();
				qpSolver._A = (*_eqFunc).Jacobian(_xf);
				qpSolver._b = -(*_eqFunc)(_xf);
				qpSolver._C = (*_ineqFunc).Jacobian(_xf);
				qpSolver._d = -(*_ineqFunc)(_xf);
			}

			return _xf;
		}

		VectorX NewtonRapshon::solve(const VectorX& x)
		{
			_xf = x;

			VectorX f;
			MatrixX J;
			while (!OptRealEqual(f = (*_func)(_xf), 0.0))
			{
				J = (*_func).Jacobian(_xf);
				_xf += J.transpose()*(J*J.transpose()).ldlt().solve(-f);
			}

			return _xf;
		}
		ProjectToFeasibleSpace::ProjectToFeasibleSpace()
		{
			_tolCon = 1e-6;
			_maxIter = 10000;
		}
		VectorX ProjectToFeasibleSpace::project(const VectorX & x0)
		{
			if (_eqConstraintFunc == NULL && _inEqConstraintFunc == NULL)
				return x0;
			else
			{
				int xN = x0.size();
				int inEqN = (*_inEqConstraintFunc)(x0).size();

				FunctionPtr augmentedFunc = FunctionPtr(new AugmentedFunction(xN, inEqN));
				static_pointer_cast<AugmentedFunction> (augmentedFunc)->_eqConstraintFunc = _eqConstraintFunc;
				static_pointer_cast<AugmentedFunction> (augmentedFunc)->_inEqConstraintFunc = _inEqConstraintFunc;
				
				NewtonRapshon nr;
				nr._func = _eqConstraintFunc;
				VectorX x(xN);
				x = nr.solve(x0);
				
				VectorX xAug(xN + inEqN);
				xAug.head(xN) = x;
				xAug.tail(inEqN) = -(*_inEqConstraintFunc)(x);
				
				VectorX updateDir(xN + inEqN);
				MatrixX J;
				VectorX f;
				int iter = 0;
				while (1)
				{
					iter++;
					updateDir.setZero();
					for (int i = 0; i < inEqN; i++)
					{
						if (xAug(xN + i) < _tolCon)
							updateDir(xN + i) = -xAug(xN + i) + 1;
					}
					f = (*augmentedFunc)(xAug);
					//cout << "x = " << endl << xAug << endl;
					//cout << "update = " << updateDir.squaredNorm() << endl << endl;
					//cout << "f = " << f.squaredNorm() << endl << endl;
					if (updateDir.squaredNorm() > 0.0 || f.squaredNorm() > _tolCon)
					{
						J = (*augmentedFunc).Jacobian(xAug);
						updateDir = updateDir - J.transpose()
							*(J*J.transpose()).ldlt().solve(J*updateDir);
						xAug += -J.transpose() * (J*J.transpose()).ldlt().solve(f) + updateDir;
					}
					else
						break;
					if (iter > _maxIter)
					{
						cout << "Projection Failed !!!" << endl;
						break;
					}
				}
				return xAug.head(xN);
			}
		}
		ProjectToFeasibleSpace::AugmentedFunction::AugmentedFunction(const int xN, const int inEqN)
		{
			_xN = xN;
			_inEqN = inEqN;
			_eqConstraintFunc = NULL;
			_inEqConstraintFunc = NULL;
		}
		VectorX ProjectToFeasibleSpace::AugmentedFunction::func(const VectorX & x) const
		{
			VectorX xF = x.head(_xN);
			VectorX s = x.tail(_inEqN);
			VectorX valEq(0);
			VectorX valInEq(0);
			if (_eqConstraintFunc != NULL)
				valEq = (*_eqConstraintFunc)(xF);
			if (_inEqConstraintFunc != NULL)
				valInEq = (*_inEqConstraintFunc)(xF) + s;
			VectorX val(valEq.size() + valInEq.size());
			val.head(valEq.size()) = valEq;
			val.tail(valInEq.size()) = valInEq;
			return val;
		}
		MatrixX ProjectToFeasibleSpace::AugmentedFunction::Jacobian(const VectorX & x) const
		{
			VectorX xF = x.head(_xN);
			MatrixX valEq(0, 0);
			MatrixX valInEq(0, 0);
			if (_eqConstraintFunc != NULL)
				valEq = (*_eqConstraintFunc).Jacobian(xF);
			if (_inEqConstraintFunc != NULL)
				valInEq = (*_inEqConstraintFunc).Jacobian(xF);
			MatrixX val(valEq.rows() + valInEq.rows(), x.size());
			val.setZero();
			val.block(0, 0, valEq.rows(), _xN) = valEq;
			val.block(valEq.rows(), 0, _inEqN, _xN) = valInEq;
			for (int i = 0; i < _inEqN; i++)
				val(valEq.rows() + i, _xN + i) = 1.0;
			return val;
		}
	}
}