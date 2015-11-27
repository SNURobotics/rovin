#pragma once

#include "Constant.h"
#include "Common.h"

#include <iostream>
#include <vector>

namespace rovin
{
	namespace Math
	{
		class OptimizationOption
		{
		public:
			enum Algorithm
			{
				defaultAlgorithm,
				InteriorPointMethod,
				SequentialQuadraticProgramming
			};

			OptimizationOption()
			{
				method = Algorithm::defaultAlgorithm;
				toleranceConstraint = 1e-8;
				maxIteration = 0;
				stepCoefficient = 0.0;
			}

			Algorithm method;
			unsigned int maxIteration;
			Real toleranceConstraint;
			Real stepCoefficient;
		};

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

		private:
			Real _eps;
		};

		class NoConstraint : public Function
		{
		public:
			NoConstraint() {}

			VectorX func(const VectorX& x) const { return VectorX(); }
		};

		static VectorX QuadraticProgrammingIneq(const MatrixX& G, const VectorX& g, const MatrixX& A, const VectorX& b, const VectorX& x0, 
			const VectorX& s0 = (VectorX()), const VectorX& lambda0 = (VectorX()), OptimizationOption options = (OptimizationOption()))
		{
			if (options.method == OptimizationOption::Algorithm::defaultAlgorithm || options.method == OptimizationOption::Algorithm::InteriorPointMethod)
			{
				if (RealEqual(options.stepCoefficient, 0.0)) options.stepCoefficient = 0.95;
				if (options.maxIteration == 0) options.maxIteration = 150;

				int n = x0.size();
				int m = b.size();

				VectorX x = x0, s = s0, lambda = lambda0;
				MatrixX S, Lambda, InvS;

				if (lambda.size() == 0)
				{
					lambda = VectorX(m);
					lambda.setOnes();
				}
				if (s.size() == 0)
				{
					s = VectorX(m);
					s.setOnes();
				}
				S = s.asDiagonal();
				Lambda = lambda.asDiagonal();

				if (G.cols() == n && G.rows() == n && g.size() == n && A.cols() == m && A.rows() == n && m == s.size() && m == lambda.size());
				else return x;

				VectorX e(m);
				e.setOnes();

				VectorX residualD, residualP, residualSLambda;
				residualD = G*x + g - A*lambda;
				residualP = s - A.transpose()*x + b;
				residualSLambda = S * Lambda * e;

				Real mu = (s.transpose()*lambda)(0) / (Real)m;

				MatrixX Gbar;
				MatrixX D;

				VectorX deltaXAff, deltaLambdaAff, deltaSAff;
				VectorX deltaX, deltaLambda, deltaS;
				MatrixX SAff, LambdaAff;

				Eigen::LDLT< MatrixX > GbarLDL;
				Real alphaAff, alpha;
				Real muAff;
				Real sigma;

				for (unsigned int iter = 0; iter < options.maxIteration; iter++)
				{
					InvS = S;
					for (int i = 0; i < m; i++)
					{
						InvS(i, i) = 1.0 / InvS(i, i);
					}
					D = InvS * Lambda;

					Gbar = G + A*D*A.transpose();
					GbarLDL = Gbar.ldlt();

					deltaXAff = GbarLDL.solve(-(residualD + A*(InvS*(residualSLambda - Lambda*residualP))));
					deltaSAff = -residualP + A.transpose()*deltaXAff;
					deltaLambdaAff = -InvS*(residualSLambda + Lambda*deltaSAff);

					alphaAff = 1.0;
					for (int i = 0; i < m; i++)
					{
						if (RealLess(deltaLambdaAff(i), 0.0))
						{
							alphaAff = min(alphaAff, -lambda(i) / deltaLambdaAff(i));
						}
					}
					for (int i = 0; i < m; i++)
					{
						if (RealLess(deltaSAff(i), 0.0))
						{
							alphaAff = min(alphaAff, -s(i) / deltaSAff(i));
						}
					}

					muAff = ((s + alphaAff*deltaSAff).transpose()*(lambda + alphaAff*deltaLambdaAff))(0) / (Real)m;
					sigma = muAff / mu;
					sigma = sigma * sigma * sigma;

					SAff = deltaSAff.asDiagonal();
					LambdaAff = deltaLambdaAff.asDiagonal();
					residualSLambda += SAff*LambdaAff*e - sigma*mu*e;
					deltaX = GbarLDL.solve(-(residualD + A*(InvS*(residualSLambda - Lambda*residualP))));
					deltaS = -residualP + A.transpose()*deltaX;
					deltaLambda = -InvS*(residualSLambda + Lambda*deltaS);

					alpha = 1.0;
					for (int i = 0; i < m; i++)
					{
						if (RealLess(deltaLambda(i), 0.0))
						{
							alpha = min(alpha, -lambda(i) / deltaLambda(i));
						}
					}
					for (int i = 0; i < m; i++)
					{
						if (RealLess(deltaS(i), 0.0))
						{
							alpha = min(alpha, -s(i) / deltaS(i));
						}
					}
					x += options.stepCoefficient*alpha*deltaX;
					s += options.stepCoefficient*alpha*deltaS;
					lambda += options.stepCoefficient*alpha*deltaLambda;

					S = s.asDiagonal();
					Lambda = lambda.asDiagonal();

					residualD = G*x + g - A*lambda;
					residualP = s - A.transpose()*x + b;
					residualSLambda = S * Lambda * e;
					mu = (s.transpose()*lambda)(0) / (Real)m;

					if (residualD.norm() >= options.toleranceConstraint || residualP.norm() >= options.toleranceConstraint);
					else break;
				}
				return x;
			}
			return x0;
		}


		static VectorX QuadraticProgramming(const MatrixX& G, const VectorX& g, const MatrixX& A, const VectorX& b, const MatrixX& C, const VectorX& d, const VectorX& x0,
			const VectorX& y0 = (VectorX()), const VectorX& s0 = (VectorX()), const VectorX& z0 = (VectorX()), OptimizationOption options = (OptimizationOption()))
		{
			std::cout << G << std::endl;
			std::cout << g << std::endl;
			std::cout << A << std::endl;
			std::cout << b << std::endl;
			std::cout << C << std::endl;
			std::cout << d << std::endl;
			if (options.method == OptimizationOption::Algorithm::defaultAlgorithm || options.method == OptimizationOption::Algorithm::InteriorPointMethod)
			{
				if (RealEqual(options.stepCoefficient, 0.0)) options.stepCoefficient = 0.95;
				if (options.maxIteration == 0) options.maxIteration = 150;

				int n = x0.size();
				int mA = b.size();
				int mC = d.size();

				VectorX x = x0, y = y0, s = s0, z = z0;
				MatrixX S, Z, InvS, InvZ;

				if (y.size() == 0)
				{
					y = VectorX(mA);
					y.setOnes();
				}
				if (z.size() == 0)
				{
					z = VectorX(mC);
					z.setOnes();
				}
				if (s.size() == 0)
				{
					s = VectorX(mC);
					s.setOnes();
				}
				S = s.asDiagonal();
				Z = z.asDiagonal();

				if (G.cols() == n && G.rows() == n && g.size() == n && A.cols() == mA && A.rows() == n && mA == y.size() && mC == s.size() && mC == z.size());
				else return x;

				VectorX e(mC);
				e.setOnes();

				VectorX residualL, residualA, residualC, residualSZ;
				residualL = G*x + g - A*y - C*z;
				residualA = A.transpose()*x - b;
				residualC = s - C.transpose()*x + d;
				residualSZ = S*Z*e;

				Real mu = (s.transpose()*z)(0) / (Real)mC;

				MatrixX Gbar(n + mA, n + mA);
				VectorX deltaX, deltaY, deltaZ, deltaS;
				VectorX delta, deltaAff;
				VectorX deltaXAff, deltaYAff, deltaZAff, deltaSAff;
				MatrixX SAff, ZAff;

				Eigen::LDLT< MatrixX > GbarLDL;
				Real alphaAff, alpha;
				Real muAff;
				Real sigma;

				VectorX tmp(n + mA);

				for (unsigned int iter = 0; iter < options.maxIteration; iter++)
				{
					InvS = S;
					for (int i = 0; i < mC; i++)
					{
						InvS(i, i) = 1.0 / InvS(i, i);
					}
					InvZ = Z;
					for (int i = 0; i < mC; i++)
					{
						InvZ(i, i) = 1.0 / InvZ(i, i);
					}
					Gbar.setZero();
					Gbar.block(0, 0, n, n) = G + C*InvS*Z*C.transpose();
					Gbar.block(0, n, n, mA) = -A;
					Gbar.block(n, 0, mA, n) = A.transpose();
					GbarLDL = Gbar.ldlt();

					tmp.block(0, 0, n, 1) = -residualL + C*(InvS*Z)*(residualC - InvZ*residualSZ);
					tmp.block(n, 0, mA, 1) = -residualA;

					deltaAff = GbarLDL.solve(tmp);
					deltaXAff = deltaAff.block(0, 0, n, 1);
					deltaYAff = deltaAff.block(0, 0, mA, 1);
					deltaZAff = -InvS*Z*C.transpose()*deltaXAff + InvS*Z*(residualC - InvZ*residualSZ);
					deltaSAff = -InvZ*(residualSZ + S*deltaZAff);

					alphaAff = 1.0;
					for (int i = 0; i < mC; i++)
					{
						if (RealLess(deltaSAff(i), 0.0))
						{
							alphaAff = min(alphaAff, -s(i) / deltaSAff(i));
						}
					}
					for (int i = 0; i < mC; i++)
					{
						if (RealLess(deltaZAff(i), 0.0))
						{
							alphaAff = min(alphaAff, -z(i) / deltaZAff(i));
						}
					}

					muAff = ((s + alphaAff*deltaSAff).transpose()*(z + alphaAff*deltaZAff))(0) / (Real)mC;
					sigma = muAff / mu;
					sigma = sigma * sigma * sigma;

					SAff = deltaSAff.asDiagonal();
					ZAff = deltaZAff.asDiagonal();
					residualSZ += SAff*ZAff*e - sigma*mu*e;

					tmp.block(0, 0, n, 1) = -residualL + C*(InvS*Z)*(residualC - InvZ*residualSZ);
					tmp.block(n, 0, mA, 1) = -residualA;

					delta = GbarLDL.solve(tmp);
					deltaX = delta.block(0, 0, n, 1);
					deltaY = delta.block(0, 0, mA, 1);
					deltaZ = -InvS*Z*C.transpose()*deltaX + InvS*Z*(residualC - InvZ*residualSZ);
					deltaS = -InvZ*(residualSZ + S*deltaZ);

					alpha = 1.0;
					for (int i = 0; i < mC; i++)
					{
						if (RealLess(deltaS(i), 0.0))
						{
							alpha = min(alpha, -s(i) / deltaS(i));
						}
					}
					for (int i = 0; i < mC; i++)
					{
						if (RealLess(deltaZ(i), 0.0))
						{
							alpha = min(alpha, -z(i) / deltaZ(i));
						}
					}

					x += options.stepCoefficient*alpha*deltaX;
					y += options.stepCoefficient*alpha*deltaY;
					s += options.stepCoefficient*alpha*deltaS;
					z += options.stepCoefficient*alpha*deltaZ;

					S = s.asDiagonal();
					Z = z.asDiagonal();

					residualL = G*x + g - A*y - C*z;
					residualA = A.transpose()*x - b;
					residualC = s - C.transpose()*x + d;
					residualSZ = S*Z*e;

					mu = (s.transpose()*z)(0) / (Real)mC;

					if (residualL.norm() >= options.toleranceConstraint || residualA.norm() >= options.toleranceConstraint || residualC.norm() >= options.toleranceConstraint);
					else break;
				}
				return x;
			}
			return x0;
		}

		static VectorX NonlinearProgramming(const Function& objf, const Function& ineqf, const Function& eqf, const VectorX& x0,
			OptimizationOption options = (OptimizationOption()))
		{
			if (options.method == OptimizationOption::Algorithm::defaultAlgorithm || options.method == OptimizationOption::Algorithm::SequentialQuadraticProgramming)
			{
				if (RealEqual(options.stepCoefficient, 0.0)) options.stepCoefficient = 0.95;
				if (options.maxIteration == 0) options.maxIteration = 500;

				int n = x0.size();
				bool inqCon = true;

				int inq = ineqf(x0).size();
				if (inq == 0)
				{
					inqCon = false;
					inq = 1;
				}
				int eq = eqf(x0).size();

				VectorX x = x0;

				VectorX d(n);

				MatrixX G(n, n);
				VectorX g(n);
				MatrixX A(n, inq);
				VectorX b(inq);

				MatrixX cJacobi(eq, n);
				for (unsigned int iter = 0; iter < options.maxIteration; iter++)
				{
					if (!inqCon)
					{
						A.setZero();
						b.setConstant(-1);
					}
					else
					{
						A = ineqf.Jacobian(x).transpose();
						b = -ineqf(x);
					}

					d.setZero();
					d = QuadraticProgramming(objf.Hessian(x)[0], objf.Jacobian(x).transpose(), eqf.Jacobian(x).transpose(), -eqf(x), A, b, d);
					x += options.stepCoefficient*d.block(0, 0, n, 1);

					if (d.block(0, 0, n, 1).norm() >= options.toleranceConstraint);
					else break;
				}
				return x;
			}
			return x0;
		}
	}
}