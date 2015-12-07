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
			void setFinalTimeAndTimeStep(const Math::Real tf, const int nStep);
			void setBoundaryCondition(const Math::VectorX& q0 = (Math::VectorX()), const Math::VectorX& qf = (Math::VectorX()),
				const Math::VectorX& qdot0 = (Math::VectorX()), const Math::VectorX& qdotf = (Math::VectorX()),
				const Math::VectorX& qddot0 = (Math::VectorX()), const Math::VectorX& qddotf = (Math::VectorX()));
			void setWayPoint(const std::vector< std::pair<Math::VectorX, Math::Real>>& wayPoints);
			void addWayPoint(std::pair<Math::VectorX, Math::Real>& wayPoint);
			void setOptimizingJointIndex(const std::vector< unsigned int >& optActiveJointIdx);

			void setConstraintRange(bool posConstraintExist = (false), bool velConstraintExist = (false), bool accConstraintExist = (false), bool jerkConstraintExist = (false));

		public:
			Model::socAssemblyPtr _socAssem;
			Model::StatePtr _defaultState;
			// Robot dof
			int _nDOF;

			// Final time
			Math::Real _tf;
			Math::Real _dt;

			///////////////////////////////
			int _nStep;

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
			std::vector< unsigned int > _optActiveJointIdx;
			int _optActiveJointDOF;

			// Joints not to optimize
			std::vector< unsigned int > _noptActiveJointIdx;
			int _noptActiveJointDOF;

			// Constraints
			bool _posConstraintExist;
			bool _velConstraintExist;
			bool _accConstraintExist;
			bool _jerkConstraintExist;

			///////////////////
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
				Effort
			};

			class SharedDID
			{
			public:
				// generate dqdp from knot vector
				SharedDID(const Model::socAssemblyPtr socAssem, const int nStep, const Math::Real dt, const Math::VectorX& knot, const Math::VectorX& noptControlPoint);

				const Math::MatrixX& getTau(const Math::VectorX& controlPoint);
				const std::vector<Math::MatrixX>& getdTaudp(const Math::VectorX& controlPoint);
				const std::vector<std::vector<Math::MatrixX>>& getd2Taudp2(const Math::VectorX& controlPoint);
				void compareControlPoint(const Math::VectorX& controlPoint);

			public:
				Model::socAssemblyPtr _socAssem;

				int _nStep;
				Math::VectorX _knot;
				Math::Real _dt;

				std::vector<Model::StatePtr> _stateTrj;
			
				std::vector<Math::MatrixX> _dqdp;
				std::vector<Math::MatrixX> _dqdotdp;
				std::vector<Math::MatrixX> _dqddotdp;
			private:
				Math::BSpline<-1, -1, -1> _optJointValSpline;
				Math::BSpline<-1, -1, -1> _optJointVelSpline;
				Math::BSpline<-1, -1, -1> _optJointAccSpline;
				Math::BSpline<-1, -1, -1> _optJointJerkSpline;

				Math::BSpline<-1, -1, -1> _noptJointValSpline;
				Math::BSpline<-1, -1, -1> _noptJointVelSpline;
				Math::BSpline<-1, -1, -1> _noptJointAccSpline;
				Math::BSpline<-1, -1, -1> _noptJointJerkSpline;

				Math::MatrixX _tau;
				std::vector<Math::MatrixX> _dtaudp;
				std::vector<std::vector<Math::MatrixX>> _d2taudp2;

				Math::VectorX _currentControlPoint;

				bool _isInitiated;
				bool _isIDUpdated;
				bool _isDIDUpdated;

				std::vector< unsigned int > _optActiveJointIdx;
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
				nonLinearInequalityConstraint();

				Math::VectorX func(const Math::VectorX& x) const;
				Math::MatrixX Jacobian(const Math::VectorX& x) const;
				std::vector<Math::MatrixX> Hessian(const Math::VectorX& x) const;

				std::shared_ptr<SharedDID> _sharedDID;
				Math::VectorX _tauMin;
				Math::VectorX _tauMax;
			};



			////////////////////// run


			void run(const ObjectiveFunctionType& objectiveType);


			////////////////////////////
			
			void setSplineCondition(const int order, const int nMiddleCP);
			void checkKnotVectorFeasibility();
			std::pair<Math::MatrixX, Math::VectorX> generateLinearEqualityConstraint(const std::vector< unsigned int >& activeJointIdx, const int activeJointDOF);
			void generateLinearEqualityConstraint();
			std::pair<Math::MatrixX, Math::VectorX> generateLinearInequalityConstraint(const std::vector< unsigned int >& activeJointIdx, const int activeJointDOF);
			void generateLinearInequalityConstraint();
			void generateNoptControlPoint();
			
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

			// Equality constraint matrix (A*x + b = 0)
			Math::MatrixX Aeq_opt;
			Math::VectorX beq_opt;
			Math::MatrixX Aeq_nopt;
			Math::VectorX beq_nopt;

			// Linear inequality constraint matrix (A*x + b <= 0)
			Math::MatrixX Aineq_opt;
			Math::VectorX bineq_opt;
			Math::MatrixX Aineq_nopt;
			Math::VectorX bineq_nopt;

		};


	}
}
