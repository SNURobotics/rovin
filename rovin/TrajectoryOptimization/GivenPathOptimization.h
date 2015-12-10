#pragma once

#include <vector>
#include <memory>

#include <rovin/Math/Constant.h>
#include <rovin/Math/Common.h>
#include <rovin/Model/Assembly.h>
#include <rovin/Dynamics/Dynamics.h>
#include <rovin/Model/State.h>
#include <rovin/Math/Spline.h>
namespace rovin
{
	namespace TrajectoryOptimization
	{
		class GivenPathOptimization
		{
		public:
			GivenPathOptimization();
			~GivenPathOptimization();

			void loadTrajectory();
			void setFinalTimeAndTimeSpan(const Math::Real tf, const int nStep);
			void setFinalTimeAndTimeSpanUsingGaussianQuadrature(const Math::Real tf, const int nStep);

		public:
			// Final time
			Math::Real _tf;
			// number of time step
			int _nStep;
			// time span points from Gaussian Quadrature (to reduce step number efficiently)
			Math::VectorX _timeSpan;
			Math::VectorX _timeSpanWeight;
		};

		class BSplineGivenPathOptimization : public GivenPathOptimization
		{
		public:
			BSplineGivenPathOptimization();
			~BSplineGivenPathOptimization();
			void setSplineCondition(const unsigned int order, const unsigned int nMiddleCP);


		public:
			// Number of middle control points & order of B-spline
			int _nMiddleCP;
			int _order;

			// Knot vector (depends on tf, boundary values, waypoints etc., should be generated after setting boundary and waypoints)
			Math::VectorX _knot;
			// Boundary control points to satisfy initial and final joint val, vel, acc
			std::vector<Math::VectorX> BoundaryCP;
			int _nInitCP;
			int _nFinalCP;

		};

	}
}

