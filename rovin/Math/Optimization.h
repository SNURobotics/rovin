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
				InteriorPointMethod
			};

			OptimizationOption()
			{
				method = Algorithm::defaultAlgorithm;
				toleranceConstraint = 1e-8;
				maxIteration = 150;
				stepCoefficient = 0.95;
			}

			Algorithm method;
			unsigned int maxIteration;
			Real toleranceConstraint;
			Real stepCoefficient;
		};

		class Function
		{
		public:
			Function()
			{
				eps = 1e-5;
			}

			VectorX operator()(const VectorX& x)
			{
				return func(x);
			}

			virtual VectorX func(const VectorX& x) = 0;
			MatrixX Jacobian(const VectorX& x);
			std::vector< MatrixX > Hessian(const VectorX& x);

		private:
			Real eps;
		};

		class NonlinearProgramming
		{

		};

		static VectorX  QuadraticProgramming(const MatrixX& G, const VectorX& g, const MatrixX& A, const VectorX& b, const VectorX& x0, 
			const VectorX& s0 = (VectorX()), const VectorX& lambda0 = (VectorX()), const OptimizationOption& options = (OptimizationOption()))
		{
			if (options.method == OptimizationOption::Algorithm::defaultAlgorithm || options.method == OptimizationOption::Algorithm::InteriorPointMethod)
			{
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

				if (G.cols() == n && G.rows() == n && g.size() == n && A.cols() == m && A.rows() == n);
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

				for (unsigned int i = 0; i < options.maxIteration; i++)
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

					if (residualD.norm() >= options.toleranceConstraint && residualP.norm() >= options.toleranceConstraint);
					else break;
				}

				return x;
			}

			return x0;
		}
	}
}