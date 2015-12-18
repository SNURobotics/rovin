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
			void setSOCRobotModel(const Model::socAssemblyPtr& socAssem, const Math::SE3& Tbase, const Math::SE3& TlastLinkToEndeffector);
			void loadToolPath(const std::string& fileName);
			void setNumberofTimeStep(const int nStep);
			void setParameters(const Math::Real feedRate, const Math::Real chordError, const Math::Real robotTimeStep, const Math::Real An, const Math::Real Jn, const Math::Real At, const Math::Real Jt);
			void truncatePath(const Math::Real curvTol);
			void setThetaGridNumber(const int thN);
			void setPathNum(const int pathNum, const Math::VectorX& qInit = (Math::VectorX()));

			// generate S-shape feed rate profile  (from Xu Du et al., "A complete S-shape feed rate scheduling approach for NURBS interpolator")
			void generateRealSwrtTime();
			Math::Real s_a(const Math::Real vs, const Math::Real ve);
			Math::Real s_d(const Math::Real vs, const Math::Real ve);
			std::vector<Math::Real> findDurationForShortBlock(const Math::Real vs, const Math::Real ve, const Math::Real vmax);
			std::vector<Math::Real> findDurationForShortBlock2(const Math::Real vs, const Math::Real ve, const Math::Real vmax);
			std::vector<Math::Real> findDurationForLongBlock(const Math::Real vs, const Math::Real ve, const Math::Real si);
			std::vector<Math::Real> findDurationForMediumBlock(const Math::Real vs, const Math::Real ve, const Math::Real si, const Math::Real eps = 1e-8);
			std::pair<std::vector<Math::Real>, std::vector<Math::Real>> getArcLengthFromTimeDuration(const std::vector<Math::Real> T, const Math::Real vs);
			class cubicFunction : public Function
			{
			public:
				cubicFunction() {}
				Math::VectorX func(const Math::VectorX& x) const;
				Real _Jt;
				Real _vn;
				Real _si;
			};
			class quadFunction : public Function
			{
			public:
				quadFunction() {}
				Math::VectorX func(const Math::VectorX& x) const;
				Real _Jt;
				Real _At;
				Real _si;
				Real _vn;
			};

			// generate constraint for sdot from curvature and feedrate
			void generateSdotMax(const int startIdx, const int endIdx);

			// Solve inverse kinematics
			void solveInverseKinematics(const int startIdx, const int endIdx, const Math::VectorX& qInit = (Math::VectorX()));
			Math::Real getManipulability(const Math::Matrix6X& J);
			void findFeasibleJointSpace(const int invIdx, const Real manipTol = 0.005, const Real invSolTol = 6.0);
			static Math::VectorX transferJointValueTest(Math::VectorX qiTrj, const Real etha = 1.0);
			static Math::VectorX transferJointValueOld(Math::VectorX qiTrj, const Real etha = 1.0);
			static Math::VectorX transferJointValue(const Math::VectorX& qiTrj);
			static Math::VectorX flipJointValue(const Math::VectorX& qTrj, Real q_bf);
			void findContinuousFeasibleSearchSpace();
			void setThetaBound(Eigen::Matrix<int, -1, -1>& feasibleRegion);
			void setThetaBound();
			
			// set boundary condtion
			void setBoundaryConditionForSdot(const Math::VectorX sdot0 = (Math::VectorX()), const Math::VectorX sdotf = (Math::VectorX()),
				const Math::VectorX sddot0 = (Math::VectorX()), const Math::VectorX sddotf = (Math::VectorX()));
			void setBoundaryConditionForTh(const VectorX th0 = (Math::VectorX()), const VectorX thf = (Math::VectorX()), const VectorX thdot0 = (Math::VectorX()), const VectorX thdotf = (Math::VectorX()),
				const VectorX thddot0 = (Math::VectorX()), const VectorX thddotf = (Math::VectorX()));
			
			// set constraint
			void setConstraint(bool velConstraintExist = (false), bool accConstraintExist = (false), bool torqueConstraintExist = (false));

			// TO DO
			std::vector<Math::VectorX> sortInvKinSol(std::vector<Math::VectorX> qCur, const std::vector<Math::VectorX> qBf);
			

		public:
			// robot model
			Model::socAssemblyPtr _socAssem;
			Math::SE3 _Tbase;
			Math::SE3 _TlastLinkToEndeffector;
			Model::StatePtr _defaultState;
			int _nDOF;
			// number of time step
			int _nStep;

			// path information
			Math::MatrixX _posTrj;
			Math::MatrixX _zAxisTrj;
			Math::VectorX _curvature;
			Math::VectorX _speed;
			
			bool _pathTruncated;

			// parameters
			Math::Real _feedRate;
			Math::Real _chordError;			// permissible tracking error
			Math::Real _robotTimeStep;
			Math::Real _criticalCurvature;
			Math::Real _An;
			Math::Real _Jn;
			Math::Real _At;
			Math::Real _Jt;
			// truncated path index
			std::vector<unsigned int> _startPathIdx;
			std::vector<unsigned int> _endPathIdx;
			int _startIdx;
			int _endIdx;
			int _pathN;
			CubicSplineInterpolation _posCubic;
			CubicSplineInterpolation _zAxisCubic;
			std::vector<CubicSplineInterpolation> _RCubic;
			std::vector<SO3>  _Ri;			// SE(3) trajectory of theta = 0 at s = s_i
			// grid number
			int _thN;		// 0 ~ 2*pi
			int _thNdouble;	// -2*pi ~ 2*pi
			int _sN;
			Math::VectorX _thSpan;
			Math::VectorX _thSpanDouble;
			Math::VectorX _sSpan;
			// arc length
			Math::Real _sf;
			CubicSplineInterpolation _realSCubic;
			CubicSplineInterpolation _sCubic;
			Math::Real _tfForThetaOnly;

			// inverse kinematics solutions
			std::vector<std::vector<Math::MatrixX>> _invKinSol;
			int _invIdx;

			// feasible search space
			Eigen::Matrix<int, -1, -1> _feasibleSet;
			int _feasibleScore;
			// splitted feasible region in (s, theta) space
			std::vector<Eigen::Matrix<int, -1, -1>> _contiFeasibleRegion;
			// largest continuous feasible region
			Eigen::Matrix<int, -1, -1> _domain;
			// feasible region for theta(s)
			Math::VectorX _thMin;
			Math::VectorX _thMax;
			bool _thBoundGenerated;
			// feasible speed
			Math::VectorX _sdotMax;

			// constraint
			bool _velConstraintExist;
			bool _torqueConstraintExist;
			bool _accConstraintExist;


			// boundary condition
			Math::VectorX _sdot0;
			Math::VectorX _sdotf;
			Math::VectorX _sddot0;
			Math::VectorX _sddotf;
			Math::VectorX _th0;
			Math::VectorX _thf;
			Math::VectorX _thdot0;
			Math::VectorX _thdotf;
			Math::VectorX _thddot0;
			Math::VectorX _thddotf;

			FunctionPtr _objectiveFunc;
			FunctionPtr _ineqFunc;
			FunctionPtr _eqFunc;
		};

		class BSplineGivenPathOptimization : public GivenPathOptimization
		{
		public:
			BSplineGivenPathOptimization();
			~BSplineGivenPathOptimization();
			
			enum ObjectiveFunctionType
			{
				Effort,
				EnergyLoss
			};

			
			class sharedVar
			{
			public:
				sharedVar(const Model::socAssemblyPtr socAssem, const int nStep, const Real sf, const Math::VectorX& sSetInvKin, const Math::VectorX& thSetInvKin, std::vector<Math::MatrixX> invKinSolData,
					const int nInitCPsdot, const int nMiddleCPsdot, const int nFinalCPsdot, const int nInitCPTh, const int nMiddleCPTh, const int nFinalCPTh,
					const Math::VectorX& knotSdot, const Math::VectorX& knotTh, 
					const std::vector<Math::VectorX>& boundaryCPsdot, const std::vector<Math::VectorX>& boundaryCPth);
				
				const Math::MatrixX& getTau(const Math::VectorX& controlPoint);
				const Math::MatrixX& getJointVal(const Math::VectorX& controlPoint);
				const Math::MatrixX& getJointVel(const Math::VectorX& controlPoint);
				const Math::MatrixX& getJointAcc(const Math::VectorX& controlPoint);
				void compareControlPoint(const Math::VectorX& controlPoint);
				void getSFromSdot();
				void getSFromSdotUsingGaussianQuadrature();
				void setTimeSpan();
				void setTimeSpanUsingGaussianQuadrature();
				// optimize theta only
				// constructor for theta only case
				sharedVar(const Model::socAssemblyPtr socAssem, const int nStep, const Real tf,
					const CubicSplineInterpolation& sCubic, const CubicSplineInterpolation& realSCubic,
					const Math::VectorX& sSetInvKin, const Math::VectorX& thSetInvKin, std::vector<Math::MatrixX> invKinSolData,
					const int nInitCPTh, const int nMiddleCPTh, const int nFinalCPTh,
					const Math::VectorX& knotTh, const std::vector<Math::VectorX>& boundaryCPth);
				void compareControlPointThetaOnly(const Math::VectorX& controlPoint);

			public:
				bool _optimizeThetaOnly;
				Model::socAssemblyPtr _socAssem;
				int _nDOF;
				std::vector<Model::StatePtr> _stateTrj;
				Math::MatrixX _tau;
				Math::MatrixX _jointVal;
				Math::MatrixX _jointVel;
				Math::MatrixX _jointAcc;
				// arc length
				Math::Real _sf;
				// number of time step
				int _nStep;
				
				// given s(t) in optimize theta only case
				CubicSplineInterpolation _sCubic;
				CubicSplineInterpolation _realSCubic;

				// earned from integration of sdot w.r.t. s
				Math::Real _tf;
				Math::VectorX _timeSpanForS;
				Math::VectorX _sSpanwrtTimeSpan;

				// gaussian quadrature for integration of sdot
				GaussianQuadrature _gaussianQuadratureSdot;
				bool _isGaussianQuadratureSdotInitialized;

				// used in integration of objective function
				Math::VectorX _timeSpanForObj;
				Math::VectorX _timeSpanWeight;

				// gaussian quadrature for integration of objective function
				GaussianQuadrature _gaussianQuadratureTime;
				bool _isGaussianQuadratureTimeInitialized;
				

				bool _isInitiated;
				bool _isIDUpdated;
				Math::VectorX _currentControlPoint;

				// inverse kinematics data
				std::vector<std::vector<Math::VectorX>> _invKinData;
				Math::VectorX _sSetInvKin;
				Math::VectorX _thSetInvKin;
				Math::BilinearInterpolation _bilinterp;
				// splines
				Math::BSpline<-1, -1, 1> _sdotSpline;
				Math::BSpline<-1, -1, 1> _thetaSpline;
				Math::VectorX _sdotCP;
				Math::VectorX _thCP;
				
				// Number of middle control points & order of B-spline
				int _nMiddleCPTh;
				int _orderTh;
				int _nMiddleCPSdot;
				int _orderSdot;
				// Knot vector (depends on tf, boundary values, waypoints etc., should be generated after setting boundary and waypoints)
				Math::VectorX _knotSdot;
				Math::VectorX _knotTh;
				// Boundary control points to satisfy initial and final joint val, vel, acc
				std::vector<Math::VectorX> _boundaryCPsdot;
				std::vector<Math::VectorX> _boundaryCPth;
				int _nInitCPsdot;
				int _nFinalCPsdot;
				int _nInitCPTh;
				int _nFinalCPTh;

			};


			void setSplineCondition(const unsigned int orderTh, const unsigned int nMiddleCPTh, const unsigned int orderS, const unsigned int nMiddleCPS, bool isKnotUniform, int par = 10);
			void setSplineConditionForThetaOnly(const unsigned int orderTh, const unsigned int nMiddleCPTh);
			void setLinearInequalityConstraint();
			void setLinearInequalityConstraintForThetaOnly();
			void getRobotTrajectory(sharedVar& sharedVar, const Math::VectorX& controlPoint);

			//////////////////////////////////////////////////////////////// run
			
			Math::VectorX run(const ObjectiveFunctionType& objectiveType);
			Math::VectorX runThetaOnly(const ObjectiveFunctionType& objectiveType);

			///////////////////////////////////////////// functions

			class effortFunction : public Math::Function
			{
			public:
				effortFunction() {}

				Math::VectorX func(const Math::VectorX& x) const;
				//Math::MatrixX Jacobian(const Math::VectorX& x) const;
				//std::vector<Math::MatrixX> Hessian(const Math::VectorX& x) const;

				std::shared_ptr<sharedVar> _sharedVar;
			};


			class energyLossFunction : public Math::Function
			{
			public:
				energyLossFunction() {}

				Math::VectorX func(const Math::VectorX& x) const;
				//Math::MatrixX Jacobian(const Math::VectorX& x) const;
				//std::vector<Math::MatrixX> Hessian(const Math::VectorX& x) const;

				std::shared_ptr<sharedVar> _sharedVar;
			};


			class inequalityConstraint : public Math::Function
			{
			public:
				inequalityConstraint() {}

				Math::VectorX func(const Math::VectorX& x) const;
				Math::MatrixX Jacobian(const Math::VectorX& x) const;
				std::vector<Math::MatrixX> Hessian(const Math::VectorX& x) const;

				Math::FunctionPtr _linearIneqConstraint;
				Math::FunctionPtr _nonLinearIneqConstraint;
			};

			class nonLinearInequalityConstraint : public Math::Function
			{
			public:
				nonLinearInequalityConstraint() {}
				enum Type
				{
					VEL,
					TORQUE,
					ACC
				};
				void loadConstraint(const Model::socAssemblyPtr& socAssem, bool velConstraintExist = (false), bool torqueConstraintExist = (false), bool accConstraintExist = (false));

				Math::VectorX func(const Math::VectorX& x) const;
				void showConstraint(const Math::VectorX& x, Type type) const;
				//Math::MatrixX Jacobian(const Math::VectorX& x) const;
				//std::vector<Math::MatrixX> Hessian(const Math::VectorX& x) const;

				std::shared_ptr<sharedVar> _sharedVar;
				Math::VectorX _tauMin;
				Math::VectorX _tauMax;
				Math::VectorX _qdotMin;
				Math::VectorX _qdotMax;
				Math::VectorX _qddotMin;
				Math::VectorX _qddotMax;
				Model::StatePtr _defaultState;
				bool _velConstraintExist;
				bool _torqueConstraintExist;
				bool _accConstraintExist;
				int _nConstraint;
			};

		public:
			
			// Number of middle control points & order of B-spline
			int _nMiddleCPTh;
			int _orderTh;
			int _nMiddleCPSdot;
			int _orderSdot;
			// Knot vector (depends on tf, boundary values, waypoints etc., should be generated after setting boundary and waypoints)
			Math::VectorX _knotSdot;
			Math::VectorX _knotTh;
			// Boundary control points to satisfy initial and final joint val, vel, acc
			std::vector<Math::VectorX> _boundaryCPsdot;
			std::vector<Math::VectorX> _boundaryCPth;
			int _nInitCPsdot;
			int _nFinalCPsdot;
			int _nInitCPTh;
			int _nFinalCPTh;

			Math::MatrixX _Aineq;
			Math::VectorX _bineq;


			//
			bool _resultFlag;
			Math::VectorX _solX;
			Math::Real _fval;
			Math::VectorX _eqConstraintVal;
			Math::VectorX _ineqConstraintVal;
			Math::Real _computationTime;
			Math::MatrixX _jointVal;
			Math::MatrixX _jointVel;
			Math::MatrixX _jointAcc;
			Math::MatrixX _jointTorque;
			Math::MatrixX _robotJointValTrj;
		};

	}
}

