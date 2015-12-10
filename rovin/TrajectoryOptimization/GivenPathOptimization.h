#pragma once

#include <vector>
#include <memory>
#include <string>
#include <rovin/Math/Constant.h>
#include <rovin/Math/Common.h>
#include <rovin/Model/Assembly.h>
#include <rovin/Dynamics/Dynamics.h>
#include <rovin/Model/State.h>
#include <rovin/Math/Spline.h>
#include <rovin/Math/GaussianQuadrature.h>
namespace rovin
{
	namespace TrajectoryOptimization
	{
		class GivenPathOptimization
		{
		public:
			GivenPathOptimization();
			~GivenPathOptimization();

			// Problem definition
			void setSOCRobotModel(const Model::socAssemblyPtr& socAssem);
			void loadToolPath(const std::string& fileName);
			void setFinalTimeAndTimeSpan(const Math::Real tf, const int nStep);
			void setFinalTimeAndTimeSpanUsingGaussianQuadrature(const Math::Real tf, const int nStep);
			void setFeedRate(const Real feedRate);
			void truncatePath(const Real curvTol);
			//void setThetaGridNumber(const int thN);
			void solveInverseKinematics(const int pathIdx, const Math::VectorX& qInit = (Math::VectorX()));
			void findFeasibleJointSpace();

			void generateMaxSpeed();


		public:
			Model::socAssemblyPtr _socAssem;
			Model::StatePtr _defaultState;
			int _nDOF;
			// Final time
			Math::Real _tf;
			// number of time step
			int _nStep;
			// time span points from Gaussian Quadrature (to reduce step number efficiently)
			Math::VectorX _timeSpan;
			Math::VectorX _timeSpanWeight;

			GaussianQuadrature _gaussianQuadrature;
			bool _gaussianQuadratureInitialized;

			// path values
			Math::MatrixX _posTrj;
			Math::MatrixX _zAxisTrj;
			Math::VectorX _curvature;

			// parameters
			Math::Real _robotTimeStep;
			Math::Real _feedRate;

			// truncated path index
			std::vector<unsigned int> _startPathIdx;
			std::vector<unsigned int> _endPathIdx;
			int _pathN;

			// grid number
			int _thN;
			int _sN;
			Math::VectorX _thSpan;
			Math::VectorX _sSpan;

			// inverse kinematics solutions
			std::vector<std::vector<Math::MatrixX>> _invKinSol;
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

