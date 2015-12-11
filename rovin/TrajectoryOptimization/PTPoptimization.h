/**
*	\file	TrajectoryOptimization.h
*	\date	2015.12.01
*	\author	Cheongjae (jchastro@gmail.com)
*	\brief	TrajectoryOptimization 클래스를 정의
*/

#pragma once

#include <vector>
#include <memory>

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

		class PointToPointOptimization
		{
		public:
			PointToPointOptimization();
			~PointToPointOptimization();

			// Problem definition
			void setSOCRobotModel(const Model::socAssemblyPtr& socAssem);
			void setFinalTimeAndTimeSpan(const Math::Real tf, const int nStep);
			void setFinalTimeAndTimeSpanUsingGaussianQuadrature(const Math::Real tf, const int nStep);
			void setBoundaryCondition(const Math::VectorX& q0 = (Math::VectorX()), const Math::VectorX& qf = (Math::VectorX()),
				const Math::VectorX& qdot0 = (Math::VectorX()), const Math::VectorX& qdotf = (Math::VectorX()),
				const Math::VectorX& qddot0 = (Math::VectorX()), const Math::VectorX& qddotf = (Math::VectorX()));
			void setWayPoint(const std::vector< std::pair<Math::VectorX, Math::Real>>& wayPoints);
			void setWayPointOnlyPosition(const std::vector<Math::Vector3>& waypoint);
			void addWayPoint(std::pair<Math::VectorX, Math::Real>& wayPoint);
			void setOptimizingJointIndex(const Math::VectorU& optActiveJointIdx);

			void setConstraintRange(bool posConstraintExist = (false), bool velConstraintExist = (false), bool torqueConstraintExist = (false), bool accConstraintExist = (false), bool jerkConstraintExist = (false));

		public:
			Model::socAssemblyPtr _socAssem;
			Model::StatePtr _defaultState;
			// Robot dof
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

			// Boundary values
			Math::VectorX _q0;
			Math::VectorX _qf;
			Math::VectorX _qdot0;
			Math::VectorX _qdotf;
			Math::VectorX _qddot0;
			Math::VectorX _qddotf;

			// Waypoints
			std::vector< std::pair<Math::VectorX, Math::Real>> _waypoint;

			// Joints to optimize
			Math::VectorU _optActiveJointIdx;
			int _optActiveJointDOF;

			// Joints not to optimize
			Math::VectorU _noptActiveJointIdx;
			int _noptActiveJointDOF;

			// Constraints
			bool _posConstraintExist;
			bool _velConstraintExist;
			bool _torqueConstraintExist;
			bool _accConstraintExist;
			//bool _jerkConstraintExist;

			bool _waypointPositionExist;
			std::vector<Math::Vector3> _waypointPosition;

			// functions in optimization
			Math::FunctionPtr _objectiveFunc;
			Math::FunctionPtr _eqFunc;
			Math::FunctionPtr _ineqFunc;

		};

		class BSplinePointToPointOptimization : public PointToPointOptimization
		{
		public:
			BSplinePointToPointOptimization();
			~BSplinePointToPointOptimization();

			enum ObjectiveFunctionType
			{
				Effort,
				EnergyLoss
			};

			enum KnotType
			{
				Uniform,
				Sided
			};

			class SharedDID
			{
			public:
				// generate dqdp from knot vector
				SharedDID(const Model::socAssemblyPtr socAssem, const int nStep, const Math::VectorX& timeSpan, const Math::VectorX& timeStepWeight, const Math::VectorX& knot, 
					const std::vector<Math::VectorX> & BoundaryCP, const int nInitCP, const int nFinalCP, const int nMiddleCP,
					const Math::VectorU& optActiveJointIdx, const Math::VectorU& noptActiveJointIdx, const int optActiveJointDOF, const int noptActiveJointDOF, const Math::VectorX& noptControlPoint,
					const std::vector<Math::MatrixX> dqdp,	std::vector<Math::MatrixX> dqdotdp, std::vector<Math::MatrixX> dqddotdp);

				
				const Math::MatrixX& getTau(const Math::VectorX& controlPoint);
				const std::vector<Math::MatrixX>& getdtaudp(const Math::VectorX& controlPoint);
				const std::vector<std::vector<Math::MatrixX>>& getd2taudp2(const Math::VectorX& controlPoint);
				void compareControlPoint(const Math::VectorX& controlPoint);

				// should be called after getTau (if not call compareControlPoint function while setting solveInverseDynamics option false)
				const Math::MatrixX& getJointVal(const Math::VectorU& activeJointIdx);
				const Math::MatrixX& getJointVel(const Math::VectorU& activeJointIdx);
				const Math::MatrixX& getJointAcc(const Math::VectorU& activeJointIdx);

			public:
				Model::socAssemblyPtr _socAssem;

				int _nStep;
				Math::VectorX _knot;
				Math::VectorX _timeSpan;
				Math::VectorX _timeSpanWeight;

				std::vector<Model::StatePtr> _stateTrj;
				std::vector<Math::MatrixX> _dqdp;
				std::vector<Math::MatrixX> _dqdotdp;
				std::vector<Math::MatrixX> _dqddotdp;
				// Boundary control points to satisfy initial and final joint val, vel, acc
				std::vector<Math::VectorX> _BoundaryCP;
				int _nInitCP;
				int _nFinalCP;
				int _nMiddleCP;
				Math::MatrixX _optCP;
				
				Math::BSpline<-1, -1, -1> _optJointValSpline;
				Math::BSpline<-1, -1, -1> _optJointVelSpline;
				Math::BSpline<-1, -1, -1> _optJointAccSpline;

				Math::BSpline<-1, -1, -1> _noptJointValSpline;
				Math::BSpline<-1, -1, -1> _noptJointVelSpline;
				Math::BSpline<-1, -1, -1> _noptJointAccSpline;

				// joint torque and derivatives
				Math::MatrixX _tau;
				std::vector<Math::MatrixX> _dtaudp;
				std::vector<std::vector<Math::MatrixX>> _d2taudp2;

				// joint value, velocity, acceleration, jerk
				Math::MatrixX _jointValTrj;
				Math::MatrixX _jointVelTrj;
				Math::MatrixX _jointAccTrj;
				Math::MatrixX _jointJerkTrj;

				Math::VectorX _currentControlPoint;

				bool _isInitiated;
				bool _isIDUpdated;
				bool _isDIDUpdated;

				Math::VectorU _optActiveJointIdx;
				Math::VectorU _noptActiveJointIdx;
				int _optActiveJointDOF;
				int _noptActiveJointDOF;
			};

			class effortFunction : public Math::Function
			{
			public:
				effortFunction() {}

				Math::VectorX func(const Math::VectorX& x) const;
				Math::MatrixX Jacobian(const Math::VectorX& x) const;
				std::vector<Math::MatrixX> Hessian(const Math::VectorX& x) const;

				std::shared_ptr<SharedDID> _sharedDID;
			};

			
			class energyLossFunction : public Math::Function
			{
			public:
				energyLossFunction() {}

				Math::VectorX func(const Math::VectorX& x) const;
				Math::MatrixX Jacobian(const Math::VectorX& x) const;
				std::vector<Math::MatrixX> Hessian(const Math::VectorX& x) const;

				std::shared_ptr<SharedDID> _sharedDID;
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
				void loadConstraint(const Model::socAssemblyPtr& socAssem, Math::VectorU& optActiveJointIdx, unsigned int optActiveDOF, bool velConstraintExist = (false), bool torqueConstraintExist = (false), bool accConstraintExist = (false));

				Math::VectorX func(const Math::VectorX& x) const;
				Math::MatrixX Jacobian(const Math::VectorX& x) const;
				std::vector<Math::MatrixX> Hessian(const Math::VectorX& x) const;

				std::shared_ptr<SharedDID> _sharedDID;
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

			class nonLinearInequalitySmallConstraint : public Math::Function
			{
			public:
				nonLinearInequalitySmallConstraint() {}
				void loadConstraint(const Model::socAssemblyPtr& socAssem);

				Math::VectorX func(const Math::VectorX& x) const;
				Math::MatrixX Jacobian(const Math::VectorX& x) const;
				std::vector<Math::MatrixX> Hessian(const Math::VectorX& x) const;

				std::shared_ptr<SharedDID> _sharedDID;
				Math::VectorX _tauMin;
				Math::VectorX _tauMax;
			};

			////////////////////////////////////////////////////////////// WAYPOINT OBJECTIVE FUNCTION
			class waypointObjectiveFunction : public Math::Function
			{
			public:
				waypointObjectiveFunction() {}

				Math::VectorX func(const Math::VectorX& x) const;
				Math::MatrixX Jacobian(const Math::VectorX& x) const;

				Real _weight;
				std::vector<Vector3> _waypoint;
				std::shared_ptr<SharedDID> _sharedDID;
			};



			////////////////////////////////////////// test function
			class effortTestFunction : public Math::Function
			{
			public:
				effortTestFunction() {}

				Math::VectorX func(const Math::VectorX& x) const;

				std::shared_ptr<SharedDID> _sharedDID;
			};

			class energyLossTestFunction : public Math::Function
			{
			public:
				energyLossTestFunction() {}

				Math::VectorX func(const Math::VectorX& x) const;

				std::shared_ptr<SharedDID> _sharedDID;
			};

			class trajectoryCheck : public Math::Function
			{
			public: 
				trajectoryCheck() {}

				Math::VectorX func(const Math::VectorX& x) const;
				Math::VectorU _activeJointIdx;
				std::shared_ptr<SharedDID> _sharedDID;
				int _timeStep;
			};


			class nonLinearInequalityTestConstraint : public Math::Function
			{
			public:
				nonLinearInequalityTestConstraint() {}

				void loadConstraint(const Model::socAssemblyPtr& socAssem, Math::VectorU& optActiveJointIdx, unsigned int optActiveDOF, bool velConstraintExist = (false), bool torqueConstraintExist = (false), bool accConstraintExist = (false));

				Math::VectorX func(const Math::VectorX& x) const;

				std::shared_ptr<SharedDID> _sharedDID;
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

			////////////////////// run


			Math::VectorX run(const ObjectiveFunctionType& objectiveType, bool withEQ = (false), bool useInitialGuess = (false));


			////////////////////////////

			void setInitialGuess(const Math::VectorX& initaloptGuess, const Math::VectorX& initalnoptGuess)
			{
				_initoptGuess = initaloptGuess;
				_initnoptGuess = initalnoptGuess;
			}
			
			void setSplineCondition(const unsigned int order, const unsigned int nMiddleCP, KnotType knotType);
			void checkKnotVectorFeasibility();
			std::pair<Math::MatrixX, Math::VectorX> generateLinearEqualityConstraint(const Math::VectorU& activeJointIdx, const int activeJointDOF);
			void generateLinearEqualityConstraint();
			
			std::pair<Math::MatrixX, Math::VectorX> generateLinearInequalityConstraint(const Math::VectorU& activeJointIdx, const int activeJointDOF);		// consider joint value only
			void generateLinearInequalityConstraint();		// consider joint value only
			
			std::pair<Math::MatrixX, Math::VectorX> generateLinearInequalityConstraint_large(const Math::VectorU& activeJointIdx, const int activeJointDOF);		// consider joint value, velocity, jerk, torque also
			void generateLinearInequalityConstraint_large();		// consider joint value, velocity, jerk, torque also
			
			void generateNoptControlPoint();
			void setdqdp();

			///// PRINT RESULT
			std::vector<Math::MatrixX> getJointTrj(const Math::VectorX& time) const;

		public:
			// Number of middle control points & order of B-spline
			int _nMiddleCP;
			int _order;

			// Knot vector (depends on tf, boundary values, waypoints etc., should be generated after setting boundary and waypoints)
			Math::VectorX _knot;
			Math::VectorX _noptControlPoint;

			// Boundary control points to satisfy initial and final joint val, vel, acc
			std::vector<Math::VectorX> BoundaryCP;
			int _nInitCP;
			int _nFinalCP;

			// Control points (concatenate boundary control points with optimizing control points)
			Math::MatrixX controlPoints;
			std::vector<Math::MatrixX> _dqdp;
			std::vector<Math::MatrixX> _dqdotdp;
			std::vector<Math::MatrixX> _dqddotdp;

			// Equality constraint matrix (A*x + b = 0)
			Math::MatrixX _Aeq_opt;
			Math::VectorX _beq_opt;
			Math::MatrixX _Aeq_nopt;
			Math::VectorX _beq_nopt;

			// Linear inequality constraint matrix (A*x + b <= 0)
			Math::MatrixX _Aineq_opt;
			Math::VectorX _bineq_opt;
			Math::MatrixX _Aineq_nopt;
			Math::VectorX _bineq_nopt;

			Math::VectorX _initoptGuess;
			Math::VectorX _initnoptGuess;
			// Solution
			Math::VectorX _solX;
			Math::Real _fval;
			Math::VectorX _eqConstraintVal;
			Math::VectorX _ineqConstraintVal;
			double _computationTime;
			bool _resultFlag;
		};


	}
}
