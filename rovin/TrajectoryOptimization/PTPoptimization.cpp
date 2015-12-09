#include "PTPoptimization.h"
#include <rovin/Model/MotorJoint.h>
#include <rovin/Math/Optimization.h>
#include <ctime>
using namespace std;
using namespace rovin::Math;
using namespace rovin::Model;

namespace rovin
{
	namespace TrajectoryOptimization
	{
		PointToPointOptimization::PointToPointOptimization()
		{
			_waypoint.resize(0);
			_optActiveJointIdx.resize(0);
			_noptActiveJointIdx.resize(0);
			_optActiveJointDOF = 0;
			_noptActiveJointDOF = 0;
		}

		PointToPointOptimization::~PointToPointOptimization()
		{

		}


		void PointToPointOptimization::setSOCRobotModel(const Model::socAssemblyPtr & socAssem)
		{
			_socAssem = socAssem;
			_defaultState = socAssem->makeState();
			_nDOF = _defaultState->getDOF(State::TARGET_JOINT::ACTIVEJOINT);
		}

		void PointToPointOptimization::setFinalTimeAndTimeStep(const Math::Real tf, const int nStep)
		{
			_tf = tf;
			_nStep = nStep;
			_timeSpan.resize(nStep);
			_timeSpanWeight.resize(nStep);
			Real _dt = tf / (Real)(nStep - 1.0);
			for (int i = 0; i < nStep; i++)
			{
				_timeSpan(i) = (Real)i * _dt;
				_timeSpanWeight(i) = _dt;
			}
			if (_timeSpan(nStep - 1) > tf)
				_timeSpan(nStep - 1) -= 1e-10;

			cout << _timeSpan << endl;
		}

		void PointToPointOptimization::setBoundaryCondition(const Math::VectorX & q0, const Math::VectorX & qf, const Math::VectorX & qdot0, const Math::VectorX & qdotf, const Math::VectorX & qddot0, const Math::VectorX & qddotf)
		{
			if ((qdot0.size() == 0 && qddot0.size() != 0) || (q0.size() == 0 && qddot0.size() != 0) || (q0.size() == 0 && qdot0.size() != 0))
				cout << "check initial boundary condition !!!" << endl;
			if (qdotf.size() == 0 && qddotf.size() != 0 || (qf.size() == 0 && qddotf.size() != 0) || (qf.size() == 0 && qdotf.size() != 0))
				cout << "check final boundary condition !!!" << endl;
			_q0 = q0;
			_qf = qf;
			_qdot0 = qdot0;
			_qdotf = qdotf;
			_qddot0 = qddot0;
			_qddotf = qddotf;
		}

		void PointToPointOptimization::setWayPoint(const std::vector<std::pair<Math::VectorX, Math::Real>>& wayPoints)
		{
			for (unsigned int i = 0; i < wayPoints.size(); i++)
			{
				if (wayPoints[i].second > 1.0)
					std::cout << "Check waypoint time !!!" << endl;
			}
			_waypoint = wayPoints;
		}

		void PointToPointOptimization::addWayPoint(std::pair<Math::VectorX, Math::Real>& wayPoint)
		{
			if (wayPoint.second > 1.0)
				std::cout << "Check waypoint time !!!" << endl;
			_waypoint.push_back(wayPoint);
		}

		void PointToPointOptimization::setOptimizingJointIndex(const VectorU& optJointIdx)
		{
			_optActiveJointIdx = optJointIdx;
			int nJoint = _socAssem->getMateList().size();
			_noptActiveJointIdx.resize(nJoint - optJointIdx.size());
			int j = 0;
			int optJointDOF = 0;
			int noptJointDOF = 0;
			int k = nJoint;
			int l = 0;
			for (int i = 0; i < nJoint; i++)
			{
				if (optJointIdx[j] == i)
				{
					j += 1;
					optJointDOF += _defaultState->getJointState(i)._dof;
					if (j >= optJointIdx.size())
					{
						k = i + 1;
						break;
					}
				}
				else
				{
					_noptActiveJointIdx(l) = i;
					l += 1;
					noptJointDOF += _defaultState->getJointState(i)._dof;
				}
			}
			for (int i = k; i < nJoint; i++)
			{
				_noptActiveJointIdx(l) = i;
				l += 1;
				noptJointDOF += _defaultState->getJointState(i)._dof;
			}
			_optActiveJointDOF = optJointDOF;
			_noptActiveJointDOF = noptJointDOF;
		}

		void PointToPointOptimization::setConstraintRange(bool posConstraintExist, bool velConstraintExist, bool torqueConstraintExist, bool accConstraintExist, bool jerkConstraintExist)
		{
			_posConstraintExist = posConstraintExist;
			_velConstraintExist = velConstraintExist;
			_torqueConstraintExist = torqueConstraintExist;
			_accConstraintExist = accConstraintExist;
			//_jerkConstraintExist = jerkConstraintExist;
		}


		BSplinePointToPointOptimization::SharedDID::SharedDID(const socAssemblyPtr socAssem, const int nStep, const VectorX& timeSpan, const VectorX& timeSpanWeight, const VectorX & knot,
			const vector<VectorX> & BoundaryCP, const int nInitCP, const int nFinalCP, const int nMiddleCP,
			const VectorU& optActiveJointIdx, const VectorU& noptActiveJointIdx, const int optActiveJointDOF, const int noptActiveJointDOF,
			const VectorX & noptControlPoint, const vector<MatrixX> dqdp, vector<MatrixX> dqdotdp, vector<MatrixX> dqddotdp)
			: _socAssem(socAssem), _nStep(nStep), _timeSpan(timeSpan), _timeSpanWeight(timeSpanWeight), _knot(knot), _optActiveJointIdx(optActiveJointIdx), _noptActiveJointIdx(noptActiveJointIdx), _optActiveJointDOF(optActiveJointDOF), _noptActiveJointDOF(noptActiveJointDOF),
			_dqdp(dqdp), _dqdotdp(dqdotdp), _dqddotdp(dqddotdp), _BoundaryCP(BoundaryCP), _nInitCP(nInitCP), _nFinalCP(nFinalCP), _nMiddleCP(nMiddleCP)
		{
			_isInitiated = false;
			StatePtr defaultState = socAssem->makeState();
			MatrixX noptCP(noptActiveJointDOF, nMiddleCP + nInitCP + nFinalCP);
			for (unsigned int i = 0, dofIdx = 0; i < noptActiveJointIdx.size(); i++)
			{
				for (unsigned int j = 0; j < defaultState->getJointState(State::TARGET_JOINT::ACTIVEJOINT, noptActiveJointIdx(i)).getDOF(); j++, dofIdx++)
				{
					for (int k = 0; k < nInitCP; k++)
					{
						noptCP(dofIdx, k) = BoundaryCP[k](noptActiveJointIdx(i));
					}
					for (int k = 0; k < nFinalCP; k++)
					{
						noptCP(dofIdx, nInitCP + nFinalCP + nMiddleCP - k - 1) = BoundaryCP[5 - k](noptActiveJointIdx(i));
					}
					for (int k = 0; k < nMiddleCP; k++)
					{
						noptCP(dofIdx, nInitCP + k) = noptControlPoint(dofIdx * nMiddleCP + k);
					}
				}
			}

			_optCP.resize(optActiveJointDOF, nMiddleCP + nInitCP + nFinalCP);
			for (int i = 0, dofIdx = 0; i < optActiveJointIdx.size(); i++)
			{
				for (unsigned int j = 0; j < defaultState->getJointState(State::TARGET_JOINT::ACTIVEJOINT, optActiveJointIdx(i)).getDOF(); j++, dofIdx++)
				{
					for (int k = 0; k < nInitCP; k++)
					{
						_optCP(dofIdx, k) = BoundaryCP[k](optActiveJointIdx(i));
					}
					for (int k = 0; k < nFinalCP; k++)
					{
						_optCP(dofIdx, nInitCP + nFinalCP + nMiddleCP - k - 1) = BoundaryCP[5 - k](optActiveJointIdx(i));
					}
				}
			}

			_noptJointValSpline = BSpline<-1, -1, -1>(knot, noptCP);
			_noptJointVelSpline = _noptJointValSpline.derivative();
			_noptJointAccSpline = _noptJointVelSpline.derivative();
			

			// set joint val, vel, acc for nopt joints
			_stateTrj = vector< StatePtr >(nStep);
			for (unsigned int i = 0; i < _stateTrj.size(); i++)
			{
				_stateTrj[i] = socAssem->makeState();
				_stateTrj[i]->setJointq(State::TARGET_JOINT::ACTIVEJOINT, _noptActiveJointIdx, _noptJointValSpline(_timeSpan(i)));
				_stateTrj[i]->setJointqdot(State::TARGET_JOINT::ACTIVEJOINT, _noptActiveJointIdx, _noptJointVelSpline(_timeSpan(i)));
				_stateTrj[i]->setJointqddot(State::TARGET_JOINT::ACTIVEJOINT, _noptActiveJointIdx, _noptJointAccSpline(_timeSpan(i)));
			}
		}

		const MatrixX& BSplinePointToPointOptimization::SharedDID::getJointVal(const VectorU& activeJointIdx)
		{
			int dof = 0;
			for (int i = 0; i < activeJointIdx.size(); i++)
				dof += _socAssem->getJointPtrByMateIndex(_stateTrj[0]->getJointID(State::TARGET_JOINT::ACTIVEJOINT, activeJointIdx(i)))->getDOF();
			_jointValTrj.resize(dof, _nStep);
			for (int i = 0; i < _nStep; i++)
				_jointValTrj.col(i) = _stateTrj[i]->getJointq(State::TARGET_JOINT::ACTIVEJOINT, activeJointIdx);

			return _jointValTrj;
		}

		const MatrixX& BSplinePointToPointOptimization::SharedDID::getJointVel(const VectorU& activeJointIdx)
		{
			int dof = 0;
			for (int i = 0; i < activeJointIdx.size(); i++)
				dof += _socAssem->getJointPtrByMateIndex(_stateTrj[0]->getJointID(State::TARGET_JOINT::ACTIVEJOINT, activeJointIdx(i)))->getDOF();
			_jointVelTrj.resize(dof, _nStep);
			for (int i = 0; i < _nStep; i++)
				_jointVelTrj.col(i) = _stateTrj[i]->getJointqdot(State::TARGET_JOINT::ACTIVEJOINT, activeJointIdx);

			return _jointVelTrj;
		}

		const MatrixX& BSplinePointToPointOptimization::SharedDID::getJointAcc(const VectorU& activeJointIdx)
		{
			int dof = 0;
			for (int i = 0; i < activeJointIdx.size(); i++)
				dof += _socAssem->getJointPtrByMateIndex(_stateTrj[0]->getJointID(State::TARGET_JOINT::ACTIVEJOINT, activeJointIdx(i)))->getDOF();
			_jointAccTrj.resize(dof, _nStep);
			for (int i = 0; i < _nStep; i++)
				_jointAccTrj.col(i) = _stateTrj[i]->getJointqddot(State::TARGET_JOINT::ACTIVEJOINT, activeJointIdx);

			return _jointAccTrj;
		}

		const MatrixX& BSplinePointToPointOptimization::SharedDID::getTau(const VectorX & controlPoint)
		{
			compareControlPoint(controlPoint);

			return _tau;
		}

		const vector<MatrixX>& BSplinePointToPointOptimization::SharedDID::getdtaudp(const Math::VectorX & controlPoint)
		{
			compareControlPoint(controlPoint);

			if (!_isDIDUpdated)
			{
				_dtaudp = vector< MatrixX >(_stateTrj.size());
				_d2taudp2 = vector< vector< MatrixX >>(_stateTrj.size());

				pair< MatrixX, std::vector< MatrixX >> DID;
				for (unsigned int i = 0; i < _stateTrj.size(); i++)
				{
					DID = Dynamics::differentiateInverseDynamics(*_socAssem, *_stateTrj[i], _dqdp[i], _dqdotdp[i], _dqddotdp[i]);
					_dtaudp[i] = DID.first;
					_d2taudp2[i] = DID.second;
				}

				_isDIDUpdated = true;
			}

			return _dtaudp;
		}

		const vector< vector< MatrixX >>& BSplinePointToPointOptimization::SharedDID::getd2taudp2(const Math::VectorX & controlPoint)
		{
			compareControlPoint(controlPoint);

			if (!_isDIDUpdated)
			{
				_dtaudp = vector< MatrixX >(_stateTrj.size());
				_d2taudp2 = vector< vector< MatrixX >>(_stateTrj.size());

				pair< MatrixX, std::vector< MatrixX >> DID;
				for (unsigned int i = 0; i < _stateTrj.size(); i++)
				{
					DID = Dynamics::differentiateInverseDynamics(*_socAssem, *_stateTrj[i], _dqdp[i], _dqdotdp[i], _dqddotdp[i]);
					_dtaudp[i] = DID.first;
					_d2taudp2[i] = DID.second;
				}

				_isDIDUpdated = true;
			}

			return _d2taudp2;
		}

		void BSplinePointToPointOptimization::SharedDID::compareControlPoint(const Math::VectorX & controlPoint)
		{
			if (_isInitiated == false || !RealEqual(_currentControlPoint, controlPoint))
			{
				_isInitiated = true;
				_currentControlPoint = controlPoint;
				_isIDUpdated = _isDIDUpdated = false;

				for (int i = 0, dofIdx = 0; i < _optActiveJointIdx.size(); i++)
				{
					for (unsigned int j = 0; j < _stateTrj[0]->getJointState(State::TARGET_JOINT::ACTIVEJOINT, _optActiveJointIdx(i)).getDOF(); j++, dofIdx++)
					{
						for (int k = 0; k < _nMiddleCP; k++)
						{
							_optCP(dofIdx, _nInitCP + k) = controlPoint(dofIdx * _nMiddleCP + k);
						}
					}
				}

				_optJointValSpline = Math::BSpline<-1, -1, -1>(_knot, _optCP);
				_optJointVelSpline = _optJointValSpline.derivative();
				_optJointAccSpline = _optJointVelSpline.derivative();

				for (unsigned int i = 0; i < _stateTrj.size(); i++)
				{
					_stateTrj[i]->setJointq(State::TARGET_JOINT::ACTIVEJOINT, _optActiveJointIdx, _optJointValSpline(_timeSpan(i)));
					_stateTrj[i]->setJointqdot(State::TARGET_JOINT::ACTIVEJOINT, _optActiveJointIdx, _optJointVelSpline(_timeSpan(i)));
					_stateTrj[i]->setJointqddot(State::TARGET_JOINT::ACTIVEJOINT, _optActiveJointIdx, _optJointAccSpline(_timeSpan(i)));
					Dynamics::solveInverseDynamics(*_socAssem, *_stateTrj[i]);
				}

				_tau = MatrixX(_stateTrj[0]->getDOF(State::TARGET_JOINT::ASSEMJOINT), _stateTrj.size());
				for (unsigned int i = 0; i < _stateTrj.size(); i++)
				{
					_tau.col(i) = _stateTrj[i]->getJointTorque(State::TARGET_JOINT::ASSEMJOINT);
				}
				_isIDUpdated = true;
			}
		}

		BSplinePointToPointOptimization::BSplinePointToPointOptimization()
		{
		}

		BSplinePointToPointOptimization::~BSplinePointToPointOptimization()
		{
		}

		Real BSplinePointToPointOptimization::run(const ObjectiveFunctionType & objectiveType)
		{
			///////////////////////////////////////// EQUALITY CONSTRAINT ///////////////////////////////////
			generateLinearEqualityConstraint();
			_eqFunc = Math::FunctionPtr(new LinearFunction());
			//static_pointer_cast<LinearFunction>(_eqFunc)->A = _Aeq_opt;
			//static_pointer_cast<LinearFunction>(_eqFunc)->b = _beq_opt;

			static_pointer_cast<LinearFunction>(_eqFunc)->A = MatrixX::Zero(1,_nMiddleCP * _optActiveJointDOF);
			static_pointer_cast<LinearFunction>(_eqFunc)->b = MatrixX::Zero(1,1);
			/////////////////////////////////////////////////////////////////////////////////////////////////

			/////////////////////////////////////// INEQUALITY CONSTRAINT ///////////////////////////////////
			_ineqFunc = Math::FunctionPtr(new inequalityConstraint());
			std::shared_ptr<LinearFunction> linearIneqFunc = std::shared_ptr<LinearFunction>(new LinearFunction());
			////////////////// LARGE(!!) linear inequality & small nonlinear inequality <<<<< slower
			//generateLinearInequalityConstraint_large();
			//std::shared_ptr<nonLinearInequalitySmallConstraint> nonLinearIneqFunc = std::shared_ptr<nonLinearInequalitySmallConstraint>(new nonLinearInequalitySmallConstraint());
			//nonLinearIneqFunc->loadConstraint(_socAssem);
			
			////////////////// small linear inequality & large nonlinear inequality <<<<< faster
			generateLinearInequalityConstraint();
			std::shared_ptr<nonLinearInequalityConstraint> nonLinearIneqFunc = std::shared_ptr<nonLinearInequalityConstraint>(new nonLinearInequalityConstraint());
			nonLinearIneqFunc->loadConstraint(_socAssem, _optActiveJointIdx, _optActiveJointDOF, _velConstraintExist, _torqueConstraintExist, _accConstraintExist);

			/////////////////////////////////////////// setting inequality constraint
			linearIneqFunc->A = _Aineq_opt;
			linearIneqFunc->b = _bineq_opt;
			static_pointer_cast<inequalityConstraint>(_ineqFunc)->_linearIneqConstraint = linearIneqFunc;
			static_pointer_cast<inequalityConstraint>(_ineqFunc)->_nonLinearIneqConstraint = nonLinearIneqFunc;
			/////////////////////////////////////////////////////////////////////////////////////////////////


			/////////////////////////////////////// NOPT CP & SET SHARED DID ///////////////////////////////////
			generateNoptControlPoint();
			setdqdp();
			shared_ptr<SharedDID> sharedDID = shared_ptr<SharedDID>(new SharedDID(_socAssem, _nStep, _timeSpan, _timeSpanWeight, _knot,
				BoundaryCP, _nInitCP, _nFinalCP, _nMiddleCP,
				_optActiveJointIdx, _noptActiveJointIdx, _optActiveJointDOF, _noptActiveJointDOF, _noptControlPoint, _dqdp, _dqdotdp, _dqddotdp));
			nonLinearIneqFunc->_sharedDID = sharedDID;
			/////////////////////////////////////////////////////////////////////////////////////////////////

			if (objectiveType == ObjectiveFunctionType::Effort)
			{
				_objectiveFunc = Math::FunctionPtr(new effortFunction());
				std::static_pointer_cast<effortFunction>(_objectiveFunc)->_sharedDID = sharedDID;
			}
			else if (objectiveType == ObjectiveFunctionType::EnergyLoss)
			{
				_objectiveFunc = Math::FunctionPtr(new energyLossFunction());
				std::static_pointer_cast<energyLossFunction>(_objectiveFunc)->_sharedDID = sharedDID;
			}

			/////////////////////////////////////////// TEST FUNCTIONS ////////////////////////////////////////////////////////
			shared_ptr<EmptyFunction> _testIneqConstFun = shared_ptr<EmptyFunction>(new EmptyFunction());
			
			shared_ptr<nonLinearInequalityTestConstraint> _testNonLinearIneqConstFun = shared_ptr<nonLinearInequalityTestConstraint>(new nonLinearInequalityTestConstraint());
			_testNonLinearIneqConstFun->_sharedDID = sharedDID;
			_testNonLinearIneqConstFun->loadConstraint(_socAssem, _optActiveJointIdx, _optActiveJointDOF, _velConstraintExist, _torqueConstraintExist, _accConstraintExist);
			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

			NonlinearOptimization nonlinearSolver;
			nonlinearSolver._objectiveFunc = _objectiveFunc;
			nonlinearSolver._eqFunc = _eqFunc;
			nonlinearSolver._ineqFunc = _ineqFunc;

			double c = clock();
			VectorX x(_optActiveJointDOF*_nMiddleCP);
			//x.setRandom();

			for (int i = 0; i < _optActiveJointDOF; i++)
			{
				for (int j = 0; j < _nMiddleCP; j++)
				{
					x(i*_nMiddleCP + j) = BoundaryCP[0](_optActiveJointIdx(i)) + (Real)(j + 1) * (BoundaryCP[5](_optActiveJointIdx(i)) - BoundaryCP[0](_optActiveJointIdx(i))) / (Real)(_nMiddleCP + 1);
				}
			}

			//cout << _Aineq_opt*x + _bineq_opt << endl;

			cout << "----------------" << endl;

			cout << (*_ineqFunc)(x) << endl;

			cout << "----------------" << endl;

			//ProjectToFeasibleSpace proj;
			//proj._eqConstraintFunc = _eqFunc;
			//proj._inEqConstraintFunc = _ineqFunc;
			//x = proj.project(x);
			cout << "---------------" << endl;
			cout << x << endl;
			cout << "----------------" << endl;
			////////////////////////////////////// TEST /////////////////////////////////////////////////////////////////////////////////////////////////

			shared_ptr<effortTestFunction> _testObjFunc = shared_ptr<effortTestFunction>(new effortTestFunction());
			_testObjFunc->_sharedDID = sharedDID;


			//shared_ptr<trajectoryCheck> _trajectoryCheck = shared_ptr<trajectoryCheck>(new trajectoryCheck());
			//_trajectoryCheck->_sharedDID = sharedDID;
			//VectorU _activeJointIdx(6);
			//_activeJointIdx << 0, 1, 2, 3, 4, 5;
			//_trajectoryCheck->_activeJointIdx = _optActiveJointIdx;
			//int timeStep = 6;
			//_trajectoryCheck->_timeStep = timeStep;

			//cout << "constraint func" << endl;
			//cout << (*nonLinearIneqFunc).func(x) << endl;
			//cout << "joint val" << endl;
			//cout << nonLinearIneqFunc->_sharedDID->getJointVal(_optActiveJointIdx) << endl;
			//cout << "joint vel" << endl;
			//cout << nonLinearIneqFunc->_sharedDID->getJointVel(_optActiveJointIdx) << endl;
			//cout << "joint acc" << endl;
			//cout << nonLinearIneqFunc->_sharedDID->getJointAcc(_optActiveJointIdx) << endl;
			//pair<MatrixX, VectorX> AB = generateLinearInequalityConstraint2(_optActiveJointIdx, _optActiveJointDOF);

			//cout << "func" << endl;
			//cout << (*_objectiveFunc).func(x) << endl;
			//cout << (*_testObjFunc).func(x) << endl;

			cout << "analytic jacobian" << endl;
			MatrixX Ja = (*_objectiveFunc).Jacobian(x);
			cout << Ja << endl;

			cout << "numerical jacobian" << endl;
			MatrixX Jnum = (*_testObjFunc).Jacobian(x);
			cout << Jnum << endl;

			//cout << "difference" << endl;
			//cout << (Ja - Jnum).squaredNorm() << endl;

			//cout << "analytic hessian" << endl;
			//vector<MatrixX> H = (*nonLinearIneqFunc).Hessian(x);
			//cout << H[0] << endl;

			//cout << "numerical hessian" << endl;
			//vector<MatrixX> Hnum = (*_testNonLinearIneqConstFun).Hessian(x);
			//cout << Hnum[0] << endl;

			///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

			x = nonlinearSolver.solve(x);
			cout << nonlinearSolver._Iter << endl;
			cout << "x : " << endl << x << endl;
			cout << "obj : " << endl << (*_objectiveFunc)(x) << endl;
			cout << "eq : " << endl << (*_eqFunc)(x) << endl;
			//cout << "ineq : " << endl << (*_ineqFunc)(x) << endl;
			cout << clock() - c  << "ms" << endl;
			return (*_objectiveFunc)(x)(0);
		}

		void BSplinePointToPointOptimization::setSplineCondition(const unsigned int order, const unsigned int nMiddleCP, BSplinePointToPointOptimization::KnotType knotType)
		{
			assert(nMiddleCP >= _waypoint.size() && "The number of control points should be bigger than the number of waypoints");
			_order = order;
			_nMiddleCP = nMiddleCP;
			int nK = order + nMiddleCP;
			unsigned int nInit = 0;
			unsigned int nFinal = 0;
			if (_q0.size() != 0)
			{
				nK += 1;  nInit += 1;
				if (_qdot0.size() != 0)
				{
					nK += 1; nInit += 1;
					if (_qddot0.size() != 0)
					{
						nK += 1;  nInit += 1;
					}
				}
			}
			if (_qf.size() != 0)
			{
				nK += 1;  nFinal += 1;
				if (_qdotf.size() != 0)
				{
					nK += 1;  nFinal += 1;
					if (_qddotf.size() != 0)
					{
						nK += 1;  nFinal += 1;
					}
				}
			}
			assert(nMiddleCP + nInit + nFinal >= order && "Number of control points should be bigger than order");

			VectorX knot(nK);
			for (unsigned int i = 0; i < order; i++)
			{
				knot(i) = 0.0;
				knot(nK - 1 - i) = _tf;
			}
			int middleStep;
			if (nInit == 0 && nFinal == 0)
			{
				middleStep = nMiddleCP - order + 1;
				for (int i = 0; i < middleStep + 1; i++)
					knot(i + order - 1) = (Real)(i) / (Real)(middleStep)* _tf;
			}
			else
			{
				if (knotType == Uniform)
				{
					middleStep = nMiddleCP + nInit + nFinal - order + 1;
					for (int i = 0; i < middleStep - 1; i++)
						knot(i + order) = (Real)(i + 1) / (Real)middleStep*_tf;
				}
				else if (knotType == Sided)
				{
					Real minT = _tf;
					Real knotStep;



					for (unsigned int i = 0; i < _waypoint.size(); i++)
					{
						if (nInit > 0)
							minT = min(minT, _waypoint[i].second);
						if (nFinal > 0)
							minT = min(minT, 1.0 - _waypoint[i].second);
					}

					middleStep = nMiddleCP - order + 1;
					if (middleStep > 0)
					{
						if (nInit > 0 && nFinal > 0)
							knotStep = _tf / (Real)(middleStep + 2);
						else
							knotStep = _tf / (Real)(middleStep + 1);

						if (knotStep < minT)
						{
							if (nInit > 0 && nFinal > 0)
							{
								for (int i = 0; i < middleStep + 1; i++)
									knot(i + order + nInit - 1) = knotStep*(Real)(i + 1);
							}
							else if (nInit > 0)
							{
								for (int i = 0; i < middleStep; i++)
									knot(i + order + nInit - 1) = knotStep*(Real)(i + 1);
							}
							else
							{
								for (int i = 0; i < middleStep; i++)
									knot(i + order) = knotStep*(Real)(i + 1);
							}
						}
						else
						{
							if (nInit > 0)
								knot(order + nInit - 1) = 0.5*minT;

							if (nFinal > 0)
								knot(nK - order - nFinal) = _tf - 0.5*minT;

							for (int i = 0; i < middleStep + 1; i++)
								knot(i + order + nInit - 1) = knot(order + nInit - 1) + (knot(nK - order - nFinal) - knot(order + nInit - 1))*(Real)i / (Real)middleStep;
						}

						for (unsigned int i = 0; i < nInit - 1; i++)
							knot(order + i) = knot(order + nInit - 1) * (Real)(i + 1) / (Real)nInit;

						for (unsigned int i = 0; i < nFinal - 1; i++)
							knot(nK - order - 1 - i) = _tf - (_tf - knot(nK - order - nFinal)) * (Real)(i + 1) / (Real)nFinal;
					}
					else
					{
						////////////////////////////
					}

				}
			}
			

			_nInitCP = nInit;
			_nFinalCP = nFinal;

			// Boundary control points for initial and final val, vel, acc
			BoundaryCP.resize(6);
			BoundaryCP[0] = _q0;
			BoundaryCP[5] = _qf;
			if (_qdot0.size() > 0)
				BoundaryCP[1] = (knot(order) - knot(order - 1)) / (Real)(order - 1) * _qdot0 + _q0;
			else
				BoundaryCP[1] = _qdot0;
			if (_qdotf.size() > 0)
				BoundaryCP[4] = -(knot(nK - order) - knot(nK - order - 1)) / (Real)(order - 1) * _qdotf + _qf;
			else
				BoundaryCP[4] = _qdotf;
			if (_qddot0.size() > 0)
				BoundaryCP[2] = (knot(order + 1) - knot(order - 1))*((knot(order) - knot(order - 1)) / (Real)(order - 1) / (Real)(order - 2)*_qddot0 + BoundaryCP[1] * (1.0 / (knot(order + 1) - knot(order - 1)) + 1.0 / (knot(order) - knot(order - 1))) - _q0 / (knot(order) - knot(order - 1)));
			else
				BoundaryCP[2] = _qddot0;
			if (_qddotf.size() > 0)
				BoundaryCP[3] = (knot(nK - order) - knot(nK - order - 2)) * ((knot(nK - order) - knot(nK - order - 1)) / (Real)(order - 1) / (Real)(order - 2)*_qddotf + BoundaryCP[4] * (1.0 / (knot(nK - order) - knot(nK - order - 2)) + 1.0 / (knot(nK - order) - knot(nK - order - 1))) - _qf / (knot(nK - order) - knot(nK - order - 1)));
			else
				BoundaryCP[3] = _qddotf;


			for (int i = 0; i < 6; i++)
			{
				cout << "boundary [" << i << "] :" << BoundaryCP[i].transpose() << endl;
			}
			_knot = knot;
		}

		void BSplinePointToPointOptimization::checkKnotVectorFeasibility()
		{
			if (_knot.size() != _order + _nMiddleCP + 4 + _waypoint.size())
				std::cout << "check length of knot vector !!!" << endl;
			for (int i = 0; i < _order; i++)
			{
				if (_knot(i) > 0.0)
					std::cout << "check initial knot vector !!!" << endl;
			}
			for (int i = 0; i < _order; i++)
			{
				if (_knot(_knot.size() - 1 - i) < _tf)
					std::cout << "check final knot vector !!!" << endl;
			}

		}

		std::pair<Math::MatrixX, Math::VectorX> BSplinePointToPointOptimization::generateLinearEqualityConstraint(const VectorU& activeJointIdx, const int activeJointDOF)
		{
			// generate equality constraint from waypoints
			MatrixX Aeq;
			VectorX beq;
			Aeq.resize(activeJointDOF*_waypoint.size(), _nMiddleCP*activeJointDOF);
			beq.resize(activeJointDOF*_waypoint.size());

			MatrixX Ni(1, _nMiddleCP + _nInitCP + _nFinalCP);
			MatrixX tempControlPoint(1, _nMiddleCP + _nInitCP + _nFinalCP);
			tempControlPoint.setZero();
			for (unsigned int i = 0; i < _waypoint.size(); i++)
			{
				for (int j = 0; j < _nMiddleCP + _nInitCP + _nFinalCP; j++)
				{
					tempControlPoint(j) = 1.0;
					BSpline<-1, -1, -1> tempSpline(_knot, tempControlPoint);
					Ni(j) = tempSpline(_tf*_waypoint[i].second)(0);
					tempControlPoint(j) = 0.0;
				}

				for (int j = 0; j < activeJointDOF; j++)
				{
					Aeq.block(i*_optActiveJointDOF + j, j*_nMiddleCP + j, 1, _nMiddleCP) = Ni.block(1, _nInitCP, 1, _nMiddleCP);
					beq(i*activeJointDOF + j) = -_waypoint[i].first(activeJointIdx[j]);
					for (int k = 0; k < _nInitCP; k++)
						beq(i*activeJointDOF + j) += Ni(1, k)*BoundaryCP[k](activeJointIdx[j]);
					for (int k = 0; k < _nFinalCP; k++)
						beq(i*activeJointDOF + j) += Ni(1, _nInitCP + _nMiddleCP + _nFinalCP - 1 - k)*BoundaryCP[5 - k](activeJointIdx[j]);
				}
			}
			pair<MatrixX, VectorX> Eq;
			Eq.first = Aeq;
			Eq.second = beq;

			return Eq;
		}

		void BSplinePointToPointOptimization::generateLinearEqualityConstraint()
		{
			_Aeq_opt.resize(_optActiveJointDOF*_waypoint.size(), _nMiddleCP*_optActiveJointDOF);
			_beq_opt.resize(_optActiveJointDOF*_waypoint.size());
			_Aeq_opt.setZero();
			_beq_opt.setZero();
			_Aeq_nopt.resize(_noptActiveJointDOF*_waypoint.size(), _nMiddleCP*_noptActiveJointDOF);
			_beq_nopt.resize(_noptActiveJointDOF*_waypoint.size());
			_Aeq_nopt.setZero();
			_beq_nopt.setZero();
			MatrixX Ni(1, _nMiddleCP + _nInitCP + _nFinalCP);
			MatrixX tempControlPoint(1, _nMiddleCP + _nInitCP + _nFinalCP);
			tempControlPoint.setZero();
			for (unsigned int i = 0; i < _waypoint.size(); i++)
			{
				for (int j = 0; j < _nMiddleCP + _nInitCP + _nFinalCP; j++)
				{
					tempControlPoint(j) = 1.0;
					BSpline<-1, -1, -1> tempSpline(_knot, tempControlPoint);
					Ni(j) = tempSpline(_tf*_waypoint[i].second)(0);
					tempControlPoint(j) = 0.0;
				}

				for (int j = 0; j < _optActiveJointDOF; j++)
				{
					_Aeq_opt.block(i*_optActiveJointDOF + j, j*_nMiddleCP, 1, _nMiddleCP) = Ni.block(0, _nInitCP, 1, _nMiddleCP);
					_beq_opt(i*_optActiveJointDOF + j) = -_waypoint[i].first(_optActiveJointIdx[j]);
					for (int k = 0; k < _nInitCP; k++)
						_beq_opt(i*_optActiveJointDOF + j) += Ni(0, k)*BoundaryCP[k](_optActiveJointIdx[j]);
					for (int k = 0; k < _nFinalCP; k++)
						_beq_opt(i*_optActiveJointDOF + j) += Ni(0, _nInitCP + _nMiddleCP + _nFinalCP - 1 - k)*BoundaryCP[5 - k](_optActiveJointIdx[j]);
				}
				for (int j = 0; j < _noptActiveJointDOF; j++)
				{
					_Aeq_nopt.block(i*_noptActiveJointDOF + j, j*_nMiddleCP, 1, _nMiddleCP) = Ni.block(0, _nInitCP, 1, _nMiddleCP);
					_beq_nopt(i*_noptActiveJointDOF + j) = -_waypoint[i].first(_noptActiveJointIdx[j]);
					for (int k = 0; k < _nInitCP; k++)
						_beq_nopt(i*_noptActiveJointDOF + j) += Ni(0, k)*BoundaryCP[k](_noptActiveJointIdx[j]);
					for (int k = 0; k < _nFinalCP; k++)
						_beq_nopt(i*_noptActiveJointDOF + j) += Ni(0, _nInitCP + _nMiddleCP + _nFinalCP - 1 - k)*BoundaryCP[5 - k](_noptActiveJointIdx[j]);
				}
			}
		}

		pair<MatrixX, VectorX> BSplinePointToPointOptimization::generateLinearInequalityConstraint(const VectorU& activeJointIdx, const int activeJointDOF)
		{
			// Inequality constraint form: A*x + b <= 0
			MatrixX Aineq;
			VectorX bineq;
			pair<MatrixX, VectorX> Ineq;
			Aineq.resize(2 * _nMiddleCP * activeJointDOF, _nMiddleCP * activeJointDOF);
			bineq.resize(2 * _nMiddleCP * activeJointDOF);
			Aineq.setZero();
			bineq.setZero();
			//Real ti;
			//Real temp;
			int joint_l;
			JointPtr tempJointPtr;
			VectorX tempUpperLimit;
			VectorX tempLowerLimit;

			// add position constraint
			for (int i = 0; i < activeJointDOF; i++)
			{
				for (int j = 0; j < _nMiddleCP; j++)
				{
					Aineq(i*_nMiddleCP + j, i*_nMiddleCP + j) = 1.0;		// less than maximum
					Aineq(_nMiddleCP*activeJointDOF + i*_nMiddleCP + j, i*_nMiddleCP + j) = -1.0;   // bigger than minimum
				}
			}
			joint_l = 0;
			for (int l = 0; l < activeJointIdx.size(); l++)
			{
				tempJointPtr = _socAssem->getJointPtrByMateIndex(_defaultState->getJointID(State::TARGET_JOINT::ACTIVEJOINT, activeJointIdx[l]));
				tempUpperLimit = tempJointPtr->getLimitPosUpper();
				tempLowerLimit = tempJointPtr->getLimitPosLower();
				for (unsigned int j = 0; j < tempJointPtr->getDOF(); j++)
				{
					for (int k = 0; k < _nMiddleCP; k++)
					{
						bineq(joint_l + k) = -tempUpperLimit(j);
						bineq(_nMiddleCP*activeJointDOF + joint_l + k) = tempLowerLimit(j);
					}
					joint_l += _nMiddleCP;
				}
			}
			Ineq.first = Aineq;
			Ineq.second = bineq;
			return Ineq;
		}

		void BSplinePointToPointOptimization::generateLinearInequalityConstraint()
		{
			pair<MatrixX, VectorX> Ineq = generateLinearInequalityConstraint(_optActiveJointIdx, _optActiveJointDOF);
			_Aineq_opt = Ineq.first;
			_bineq_opt = Ineq.second;
			Ineq = generateLinearInequalityConstraint(_noptActiveJointIdx, _noptActiveJointDOF);
			_Aineq_nopt = Ineq.first;
			_bineq_nopt = Ineq.second;
		}

		pair<MatrixX, VectorX> BSplinePointToPointOptimization::generateLinearInequalityConstraint_large(const VectorU& activeJointIdx, const int activeJointDOF)
		{
			// Inequality constraint form: A*x + b <= 0
			int nPosConstraint = 0;
			int nStartVel = 0;
			int nStartAcc = 0;
			int nStartJerk = 0;

			if (_posConstraintExist)
			{
				nPosConstraint = 2;
				nStartVel += nPosConstraint * _nMiddleCP * activeJointDOF;
				nStartAcc += nPosConstraint * _nMiddleCP * activeJointDOF;
				nStartJerk += nPosConstraint * _nMiddleCP * activeJointDOF;
			}

			int nConstraint = 0;
			if (_velConstraintExist)
			{
				nConstraint += 2;
				nStartAcc += 2 * _nStep * activeJointDOF;
				nStartJerk += 2 * _nStep * activeJointDOF;
			}

			if (_accConstraintExist)
			{
				nConstraint += 2;
				nStartJerk += 2 * _nStep * activeJointDOF;
			}

			//if (_jerkConstraintExist)
			//	nConstraint += 2;
			MatrixX Aineq;
			VectorX bineq;
			pair<MatrixX, VectorX> Ineq;
			Aineq.resize((nConstraint * _nStep + nPosConstraint * _nMiddleCP)*activeJointDOF, _nMiddleCP*activeJointDOF);
			bineq.resize((nConstraint * _nStep + nPosConstraint * _nMiddleCP)*activeJointDOF);
			Aineq.setZero();
			bineq.setZero();
			Real ti;
			Real temp;
			int joint_l;
			int jointID;
			JointPtr tempJointPtr;
			VectorX tempUpperLimit;
			VectorX tempLowerLimit;
			MatrixX dNi(1, _nMiddleCP + _nInitCP + _nFinalCP);
			MatrixX ddNi(1, _nMiddleCP + _nInitCP + _nFinalCP);
			MatrixX dddNi(1, _nMiddleCP + _nInitCP + _nFinalCP);
			MatrixX tempControlPoint(1, _nMiddleCP + _nInitCP + _nFinalCP);
			tempControlPoint.setZero();

			// add position constraint
			if (_posConstraintExist)
			{
				for (int i = 0; i < activeJointDOF; i++)
				{
					for (int j = 0; j < _nMiddleCP; j++)
					{
						Aineq(i*_nMiddleCP + j, i*_nMiddleCP + j) = 1.0;		// less than maximum
						Aineq(_nMiddleCP*activeJointDOF + i*_nMiddleCP + j, i*_nMiddleCP + j) = -1.0;   // bigger than minimum
					}
				}
				joint_l = 0;
				for (int l = 0; l < activeJointIdx.size(); l++)
				{
					tempJointPtr = _socAssem->getJointPtrByMateIndex(_defaultState->getJointID(State::TARGET_JOINT::ACTIVEJOINT, activeJointIdx[l]));
					tempUpperLimit = tempJointPtr->getLimitPosUpper();
					tempLowerLimit = tempJointPtr->getLimitPosLower();
					for (unsigned int j = 0; j < tempJointPtr->getDOF(); j++)
					{
						for (int k = 0; k < _nMiddleCP; k++)
						{
							bineq(joint_l + k) = -tempUpperLimit(j);
							bineq(_nMiddleCP*activeJointDOF + joint_l + k) = tempLowerLimit(j);
						}
						joint_l += _nMiddleCP;
					}
				}
			}

			if (nConstraint > 0)
			{
				for (int i = 0; i < _nStep; i++)
				{
					for (int j = 0; j < _nMiddleCP + _nInitCP + _nFinalCP; j++)
					{
						tempControlPoint(j) = 1.0;
						BSpline<-1, -1, -1> tempSpline(_knot, tempControlPoint);
						tempControlPoint(j) = 0.0;
						BSpline<-1, -1, -1> dtempSpline = tempSpline.derivative();
						dNi(j) = dtempSpline(_timeSpan(i))(0);
						BSpline<-1, -1, -1> ddtempSpline = dtempSpline.derivative();
						ddNi(j) = ddtempSpline(_timeSpan(i))(0);
						//if (_jerkConstraintExist)
						//{
						//	BSpline<-1, -1, -1> dddtempSpline = ddtempSpline.derivative();
						//	dddNi(j) = dddtempSpline(ti)(0);
						//}
					}

					// add velocity constraint
					if (_velConstraintExist)
					{
						for (int j = 0, actDofIdx = 0; j < activeJointIdx.size(); j++)
						{
							jointID = _defaultState->getJointID(State::TARGET_JOINT::ACTIVEJOINT, activeJointIdx(j));
							for (unsigned int l = 0; l < _socAssem->getJointPtrByMateIndex(jointID)->getDOF(); l++, actDofIdx++)
							{
								Aineq.block(nStartVel + activeJointDOF*i + actDofIdx, _nMiddleCP*actDofIdx, 1, _nMiddleCP) = dNi.block(0, _nInitCP, 1, _nMiddleCP);		// less than maximum
								Aineq.block(nStartVel + activeJointDOF * _nStep + activeJointDOF*i + actDofIdx, _nMiddleCP*actDofIdx, 1, _nMiddleCP) = -dNi.block(0, _nInitCP, 1, _nMiddleCP);   // bigger than minimum
								for (int k = 0; k < _nInitCP; k++)
								{
									temp = dNi(k)*BoundaryCP[k](activeJointIdx[j] + l);
									bineq(nStartVel + activeJointDOF*i + actDofIdx) += temp;
									bineq(nStartVel + activeJointDOF * _nStep + activeJointDOF*i + actDofIdx) -= temp;
								}
								for (int k = 0; k < _nFinalCP; k++)
								{
									temp = dNi(_nInitCP + _nMiddleCP + _nFinalCP - 1 - k)*BoundaryCP[k](activeJointIdx[j] + l);
									bineq(nStartVel + activeJointDOF*i + actDofIdx) += temp;
									bineq(nStartVel + activeJointDOF * _nStep + activeJointDOF*i + actDofIdx) -= temp;
								}
							}
						}
					}
					// add acceleration constraint
					if (_accConstraintExist)
					{
						for (int j = 0, activeDofIdx = 0; j < activeJointDOF; j++)
						{
							jointID = _defaultState->getJointID(State::TARGET_JOINT::ACTIVEJOINT, activeJointIdx(j));
							for (unsigned int l = 0; l < _socAssem->getJointPtrByMateIndex(jointID)->getDOF(); l++, activeDofIdx++)
							{
								Aineq.block(nStartAcc + activeJointDOF*i + activeDofIdx, _nMiddleCP*activeDofIdx, 1, _nMiddleCP) = ddNi.block(0, _nInitCP, 1, _nMiddleCP);		// less than maximum
								Aineq.block(nStartAcc + activeJointDOF * _nStep + activeJointDOF*i + activeDofIdx, _nMiddleCP*activeDofIdx, 1, _nMiddleCP) = -ddNi.block(0, _nInitCP, 1, _nMiddleCP);   // bigger than minimum
																																												  //bineq(nStartVel + activeJointDOF*i + j) = getVelocityLimit;
								for (int k = 0; k < _nInitCP; k++)
								{
									temp = ddNi(k)*BoundaryCP[k](activeJointIdx[j]);
									bineq(nStartAcc + activeJointDOF*i + activeDofIdx) += temp;
									bineq(nStartAcc + activeJointDOF * _nStep + activeJointDOF*i + activeDofIdx) -= temp;
								}
								for (int k = 0; k < _nFinalCP; k++)
								{
									temp = ddNi(_nInitCP + _nMiddleCP + _nFinalCP - 1 - k)*BoundaryCP[k](activeJointIdx[j]);
									bineq(nStartAcc + activeJointDOF*i + activeDofIdx) += temp;
									bineq(nStartAcc + activeJointDOF * _nStep + activeJointDOF*i + activeDofIdx) -= temp;
								}
							}
						}
					}
					//// add jerk constraint
					//if (_jerkConstraintExist)
					//{
					//	for (int j = 0; j < activeJointDOF; j++)
					//	{
					//		Aineq.block(nStartJerk + activeJointDOF*i + j, _nMiddleCP*j, 1, _nMiddleCP) = dddNi.block(0, _nInitCP, 1, _nMiddleCP);		// less than maximum
					//		Aineq.block(nStartJerk + activeJointDOF * _nStep + activeJointDOF*i + j, _nMiddleCP*j, 1, _nMiddleCP) = -dddNi.block(0, _nInitCP, 1, _nMiddleCP);   // bigger than minimum
					//																																							//bineq(nStartVel + activeJointDOF*i + j) = getVelocityLimit;
					//		for (int k = 0; k < _nInitCP; k++)
					//		{
					//			temp = dddNi(k)*BoundaryCP[k](activeJointIdx[j]);
					//			bineq(nStartJerk + activeJointDOF*i + j) += temp;
					//			bineq(nStartJerk + activeJointDOF * _nStep + activeJointDOF*i + j) -= temp;
					//		}
					//		for (int k = 0; k < _nFinalCP; k++)
					//		{
					//			temp = dddNi(_nInitCP + _nMiddleCP + _nFinalCP - 1 - k)*BoundaryCP[k](activeJointIdx[j]);
					//			bineq(nStartJerk + activeJointDOF*i + j) += temp;
					//			bineq(nStartJerk + activeJointDOF * _nStep + activeJointDOF*i + j) -= temp;
					//		}
					//	}
					//}
				}

				if (_velConstraintExist)
				{
					joint_l = 0;
					for (int l = 0; l < activeJointIdx.size(); l++)
					{
						tempJointPtr = _socAssem->getJointPtrByMateIndex(_defaultState->getJointID(State::TARGET_JOINT::ACTIVEJOINT, activeJointIdx(l)));
						tempUpperLimit = tempJointPtr->getLimitVelUpper();
						tempLowerLimit = tempJointPtr->getLimitVelLower();
						for (unsigned int j = 0; j < tempJointPtr->getDOF(); j++)
						{
							for (int i = 0; i < _nStep; i++)
							{
								bineq(nStartVel + activeJointDOF*i + joint_l + j) -= tempUpperLimit(j);
								bineq(nStartVel + activeJointDOF * _nStep + activeJointDOF*i + joint_l + j) += tempLowerLimit(j);
							}
						}
						joint_l += tempJointPtr->getDOF();
					}
				}

				if (_accConstraintExist)
				{
					joint_l = 0;
					for (int l = 0; l < activeJointIdx.size(); l++)
					{
						tempJointPtr = _socAssem->getJointPtrByMateIndex(_defaultState->getJointID(State::TARGET_JOINT::ACTIVEJOINT, activeJointIdx(l)));
						tempUpperLimit = tempJointPtr->getLimitAccUpper();
						tempLowerLimit = tempJointPtr->getLimitAccLower();
						for (unsigned int j = 0; j < tempJointPtr->getDOF(); j++)
						{
							for (int i = 0; i < _nStep; i++)
							{
								bineq(nStartAcc + activeJointDOF*i + joint_l + j) -= tempUpperLimit(j);
								bineq(nStartAcc + activeJointDOF * _nStep + activeJointDOF*i + joint_l + j) += tempLowerLimit(j);
							}
						}
						joint_l += tempJointPtr->getDOF();
					}
				}

				//if (_jerkConstraintExist)
				//{
				//	joint_l = 0;
				//	for (unsigned int l = 0; l < activeJointIdx.size(); l++)
				//	{
				//		tempJointPtr = _socAssem->getJointPtrByMateIndex(_defaultState->getJointID(State::TARGET_JOINT::ACTIVEJOINT, activeJointIdx(l)));
				//		tempUpperLimit = tempJointPtr->getLimitJerkUpper();
				//		tempLowerLimit = tempJointPtr->getLimitJerkLower();
				//		for (unsigned int j = 0; j < tempJointPtr->getDOF(); j++)
				//		{
				//			for (int i = 0; i < _nStep; i++)
				//			{
				//				bineq(nStartJerk + activeJointDOF*i + joint_l + j) -= tempUpperLimit(j);
				//				bineq(nStartJerk + activeJointDOF * _nStep + activeJointDOF*i + joint_l + j) += tempLowerLimit(j);
				//			}
				//		}
				//		joint_l += tempJointPtr->getDOF();
				//	}
				//}

			}
			Ineq.first = Aineq;
			Ineq.second = bineq;
			return Ineq;
		}


		void BSplinePointToPointOptimization::generateLinearInequalityConstraint_large()
		{
			pair<MatrixX, VectorX> Ineq = generateLinearInequalityConstraint_large(_optActiveJointIdx, _optActiveJointDOF);
			_Aineq_opt = Ineq.first;
			_bineq_opt = Ineq.second;
			Ineq = generateLinearInequalityConstraint_large(_noptActiveJointIdx, _noptActiveJointDOF);
			_Aineq_nopt = Ineq.first;
			_bineq_nopt = Ineq.second;
		}

		void BSplinePointToPointOptimization::generateNoptControlPoint()
		{

			_noptControlPoint.resize(_nMiddleCP*_noptActiveJointDOF);
			for (int i = 0; i < _noptActiveJointDOF; i++)
			{
				for (int j = 0; j < _nMiddleCP; j++)
				{
					_noptControlPoint(i*_nMiddleCP + j) = BoundaryCP[0](_noptActiveJointIdx(i)) + (Real)(j + 1) * (BoundaryCP[5](_noptActiveJointIdx(i)) - BoundaryCP[0](_noptActiveJointIdx(i))) / (Real)(_nMiddleCP + 1);
				}
			}
			
			cout << "noptCP" << endl;
			cout << _noptControlPoint.transpose() << endl;

			ProjectToFeasibleSpace proj;
			FunctionPtr linearEq = FunctionPtr(new LinearFunction());
			FunctionPtr linearInEq = FunctionPtr(new LinearFunction());
			static_pointer_cast<LinearFunction> (linearEq)->A = _Aeq_nopt;
			static_pointer_cast<LinearFunction> (linearEq)->b = _beq_nopt;

			static_pointer_cast<LinearFunction> (linearInEq)->A = _Aineq_nopt;
			static_pointer_cast<LinearFunction> (linearInEq)->b = _bineq_nopt;

			if (_noptActiveJointDOF > 0)
			{
				proj._eqConstraintFunc = linearEq;
				proj._inEqConstraintFunc = linearInEq;
			}
			
			
			
			//_noptControlPoint = proj.project(_noptControlPoint);
		}

		void BSplinePointToPointOptimization::setdqdp()
		{
			// calculate dqdp, dqdotdp, dqddotdp
			unsigned int totalDOF = _defaultState->getDOF(State::TARGET_JOINT::ASSEMJOINT);
			_dqdp.resize(_nStep);
			_dqdotdp.resize(_nStep);
			_dqddotdp.resize(_nStep);
			for (int i = 0; i < _nStep; i++)
			{
				_dqdp[i] = MatrixX::Zero(totalDOF, _nMiddleCP*_optActiveJointDOF);
				_dqdotdp[i] = MatrixX::Zero(totalDOF, _nMiddleCP*_optActiveJointDOF);
				_dqddotdp[i] = MatrixX::Zero(totalDOF, _nMiddleCP*_optActiveJointDOF);
			}


			MatrixX tempControlPoint(1, _nMiddleCP + _nInitCP + _nFinalCP);
			tempControlPoint.setZero();
			VectorX Ni(1);
			VectorX dNi(1);
			VectorX ddNi(1);
			Real ti;
			unsigned int dofIdx;
			VectorU accumulatedDOF(_socAssem->getMateList().size());
			accumulatedDOF(0) = _socAssem->getJointPtrByMateIndex(0)->getDOF();
			for (int jointID = 1; jointID < accumulatedDOF.size(); jointID++)
			{
				accumulatedDOF(jointID) = accumulatedDOF(jointID - 1) + _socAssem->getJointPtrByMateIndex(jointID)->getDOF();
			}

			//cout << accumulatedDOF << endl;

			unsigned int jointID;
			for (int j = 0; j < _nMiddleCP; j++)
			{
				tempControlPoint(_nInitCP + j) = 1.0;
				BSpline<-1, -1, -1> tempSpline(_knot, tempControlPoint);
				tempControlPoint(_nInitCP + j) = 0.0;
				BSpline<-1, -1, -1> dtempSpline = tempSpline.derivative();
				BSpline<-1, -1, -1> ddtempSpline = dtempSpline.derivative();

				for (int i = 0; i < _nStep; i++)
				{
					Ni = tempSpline(_timeSpan(i));
					dNi = dtempSpline(_timeSpan(i));
					ddNi = ddtempSpline(_timeSpan(i));
					for (int k = 0, optActiveDofIdx = 0; k < _optActiveJointIdx.size(); k++)
					{
						jointID = _defaultState->getJointID(State::TARGET_JOINT::ACTIVEJOINT, _optActiveJointIdx[k]);
						for (unsigned int l = 0; l < _socAssem->getJointPtrByMateIndex(jointID)->getDOF(); l++, optActiveDofIdx++)
						{
							dofIdx = accumulatedDOF(jointID) + l - 1;
							_dqdp[i](dofIdx, _nMiddleCP*optActiveDofIdx + j) = Ni(0);
							_dqdotdp[i](dofIdx, _nMiddleCP*optActiveDofIdx + j) = dNi(0);
							_dqddotdp[i](dofIdx, _nMiddleCP*optActiveDofIdx + j) = ddNi(0);
						}
					}
				}
			}
		}

		Math::VectorX BSplinePointToPointOptimization::effortFunction::func(const Math::VectorX & x) const
		{
			const MatrixX& tauTrj = _sharedDID->getTau(x);

			VectorX val(1);
			val(0) = 0;
			for (int i = 0; i < tauTrj.cols(); i++)
				val(0) += tauTrj.col(i).squaredNorm()*_sharedDID->_timeSpanWeight(i);
			return val;
		}

		Math::MatrixX BSplinePointToPointOptimization::effortFunction::Jacobian(const Math::VectorX & x) const
		{
			const MatrixX& tauTrj = _sharedDID->getTau(x);

			const vector<MatrixX>& dtaudpTrj = _sharedDID->getdtaudp(x);
			Math::MatrixX val(1, x.size());
			val.setZero();
			for (int i = 0; i < tauTrj.cols(); i++)
			{
				val += tauTrj.col(i).transpose()*dtaudpTrj[i] * _sharedDID->_timeSpanWeight(i);
			}

			return 2.0*val;
		}

		vector<Math::MatrixX> BSplinePointToPointOptimization::effortFunction::Hessian(const Math::VectorX & x) const
		{
			const MatrixX& tauTrj = _sharedDID->getTau(x);
			const vector<MatrixX>& dtaudpTrj = _sharedDID->getdtaudp(x);
			const vector<vector<MatrixX>>& d2taudp2Trj = _sharedDID->getd2taudp2(x);
			vector<Math::MatrixX> val(1, MatrixX::Zero(x.size(),x.size()));
			for (int i = 0; i < tauTrj.cols(); i++)
			{
				for (int j = 0; j < tauTrj.rows(); j++)
					val[0] += 2.0 * _sharedDID->_timeSpanWeight(i) * tauTrj(j,i)*d2taudp2Trj[i][j];
				val[0] += 2.0 * _sharedDID->_timeSpanWeight(i) * dtaudpTrj[i].transpose()*dtaudpTrj[i];
			}
			
			return val;
		}

		Math::VectorX BSplinePointToPointOptimization::effortTestFunction::func(const Math::VectorX & x) const
		{
			const MatrixX& tauTrj = _sharedDID->getTau(x);

			VectorX val(1);
			val(0) = 0;
			for (int i = 0; i < tauTrj.cols(); i++)
				val(0) += tauTrj.col(i).squaredNorm()*_sharedDID->_timeSpanWeight(i);
			return val;
		}

		Math::VectorX BSplinePointToPointOptimization::energyLossFunction::func(const Math::VectorX & x) const
		{
			const MatrixX& tauTrj = _sharedDID->getTau(x);
			shared_ptr<MotorJoint> tempJointPtr;
			Real voltage;
			Real current;
			VectorX val(1);
			val(0) = 0;
			for (int i = 0; i < tauTrj.cols(); i++)
			{
				for (unsigned int j = 0, dofIdx = 0; j < _sharedDID->_socAssem->getMateList().size(); j++)
				{
					tempJointPtr = static_pointer_cast<MotorJoint>(_sharedDID->_socAssem->getJointPtrByMateIndex(j));
					for (unsigned int k = 0; k < tempJointPtr->getDOF(); k++, dofIdx++)
					{
						current = 1.0 / (tempJointPtr->getMotorConstant() * tempJointPtr->getGearRatio()) * tauTrj(dofIdx, i)
							+ tempJointPtr->getRotorInertia() * tempJointPtr->getGearRatio() / tempJointPtr->getMotorConstant() * _sharedDID->_stateTrj[i]->getJointStateByMateIndex(j).getqddot()(k);
						voltage = current * tempJointPtr->getResistance() + tempJointPtr->getBackEMFConstant() * tempJointPtr->getGearRatio() * _sharedDID->_stateTrj[i]->getJointStateByMateIndex(j).getqdot()(k);
						val(0) += _sharedDID->_timeSpanWeight(i) * max(current * voltage, 0.0);
						
					}
				}
			}
			return val;
		}

		Math::MatrixX BSplinePointToPointOptimization::energyLossFunction::Jacobian(const Math::VectorX & x) const
		{
			const MatrixX& tauTrj = _sharedDID->getTau(x);
			const vector<MatrixX>& dtaudpTrj = _sharedDID->getdtaudp(x);
			shared_ptr<MotorJoint> tempJointPtr;
			Real voltage;
			Real current;
			Math::MatrixX val(1, x.size());
			val.setZero();
			Math::MatrixX dcurrentdp(1, x.size());
			for (int i = 0; i < tauTrj.cols(); i++)
			{
				for (unsigned int j = 0, dofIdx = 0; j < _sharedDID->_socAssem->getMateList().size(); j++)
				{
					tempJointPtr = static_pointer_cast<MotorJoint>(_sharedDID->_socAssem->getJointPtrByMateIndex(j));
					for (unsigned int k = 0; k < tempJointPtr->getDOF(); k++, dofIdx++)
					{
						current = 1.0 / (tempJointPtr->getMotorConstant() * tempJointPtr->getGearRatio()) * tauTrj(dofIdx, i)
							+ tempJointPtr->getRotorInertia() * tempJointPtr->getGearRatio() / tempJointPtr->getMotorConstant() * _sharedDID->_stateTrj[i]->getJointStateByMateIndex(j).getqddot()(k);
						voltage = current * tempJointPtr->getResistance() + tempJointPtr->getBackEMFConstant() * tempJointPtr->getGearRatio() * _sharedDID->_stateTrj[i]->getJointStateByMateIndex(j).getqdot()(k);
						
						if (current * voltage > 0.0)
						{
							dcurrentdp = 1.0 / (tempJointPtr->getMotorConstant() * tempJointPtr->getGearRatio()) * dtaudpTrj[i].row(dofIdx)
								+ tempJointPtr->getRotorInertia() * tempJointPtr->getGearRatio() / tempJointPtr->getMotorConstant() * _sharedDID->_dqddotdp[i].row(dofIdx);
							val += ((current * tempJointPtr->getResistance() + voltage) * dcurrentdp 
								+ current * tempJointPtr->getBackEMFConstant()*tempJointPtr->getGearRatio() * _sharedDID->_dqdotdp[i].row(dofIdx)) * _sharedDID->_timeSpanWeight(i);
						}

					}
				}
			}

			return val;
		}

		vector<Math::MatrixX> BSplinePointToPointOptimization::energyLossFunction::Hessian(const Math::VectorX & x) const
		{

			const MatrixX& tauTrj = _sharedDID->getTau(x);
			const vector<MatrixX>& dtaudpTrj = _sharedDID->getdtaudp(x);
			const vector<vector<MatrixX>>& d2taudp2Trj = _sharedDID->getd2taudp2(x);
			shared_ptr<MotorJoint> tempJointPtr;
			Real voltage;
			Real current;
			vector<Math::MatrixX> val(1, MatrixX(x.size(), x.size()));
			val[0].setZero();
			MatrixX dcurrentdp(1, x.size());
			MatrixX dvoltagedp(1, x.size());
			for (int i = 0; i < tauTrj.cols(); i++)
			{
				for (unsigned int j = 0, dofIdx = 0; j < _sharedDID->_socAssem->getMateList().size(); j++)
				{
					tempJointPtr = static_pointer_cast<MotorJoint>(_sharedDID->_socAssem->getJointPtrByMateIndex(j));
					for (unsigned int k = 0; k < tempJointPtr->getDOF(); k++, dofIdx++)
					{
						current = 1.0 / (tempJointPtr->getMotorConstant() * tempJointPtr->getGearRatio()) * tauTrj(dofIdx, i)
							+ tempJointPtr->getRotorInertia() * tempJointPtr->getRotorInertia() * tempJointPtr->getGearRatio() / tempJointPtr->getMotorConstant() * _sharedDID->_stateTrj[i]->getJointStateByMateIndex(j).getqddot()(k);
						voltage = current * tempJointPtr->getResistance() + tempJointPtr->getBackEMFConstant() * tempJointPtr->getGearRatio() * _sharedDID->_stateTrj[i]->getJointStateByMateIndex(j).getqdot()(k);

						if (current * voltage > 0.0)
						{
							dcurrentdp = 1.0 / (tempJointPtr->getMotorConstant() * tempJointPtr->getGearRatio()) * dtaudpTrj[i].row(dofIdx)
								+ tempJointPtr->getRotorInertia() * tempJointPtr->getRotorInertia() / tempJointPtr->getMotorConstant() * _sharedDID->_dqddotdp[i].row(dofIdx);
							dvoltagedp = tempJointPtr->getResistance() * dcurrentdp
								+ tempJointPtr->getBackEMFConstant()*tempJointPtr->getGearRatio() * _sharedDID->_dqdotdp[i].row(dofIdx);
							val[0] += _sharedDID->_timeSpanWeight(i) * ((current * tempJointPtr->getResistance() + voltage) / (tempJointPtr->getMotorConstant() * tempJointPtr->getGearRatio()) * d2taudp2Trj[i][dofIdx]
								+ dcurrentdp.transpose() * dvoltagedp + dvoltagedp.transpose() * dcurrentdp);
						}

					}
				}
			}
			return val;
		}

		Math::VectorX BSplinePointToPointOptimization::inequalityConstraint::func(const Math::VectorX & x) const
		{
			VectorX linearIneq = (*_linearIneqConstraint)(x);
			VectorX nonlinearIneq = (*_nonLinearIneqConstraint)(x);
			VectorX val(linearIneq.size() + nonlinearIneq.size());
			for (int i = 0; i < linearIneq.size(); i++)
				val(i) = linearIneq(i);
			for (int i = 0; i < nonlinearIneq.size(); i++)
				val(linearIneq.size() + i) = nonlinearIneq(i);

		//	cout << val << endl;

			return val;
		}

		Math::MatrixX BSplinePointToPointOptimization::inequalityConstraint::Jacobian(const Math::VectorX & x) const
		{
			MatrixX linearIneq = (*_linearIneqConstraint).Jacobian(x);
			MatrixX nonlinearIneq = (*_nonLinearIneqConstraint).Jacobian(x);
			MatrixX val(linearIneq.rows() + nonlinearIneq.rows(), x.size());
			val.block(0, 0, linearIneq.rows(), x.size()) = linearIneq;
			val.block(linearIneq.rows(), 0, nonlinearIneq.rows(), x.size()) = nonlinearIneq;
			return val;
		}

		vector<Math::MatrixX> BSplinePointToPointOptimization::inequalityConstraint::Hessian(const Math::VectorX & x) const
		{
			vector<MatrixX> val = (*_linearIneqConstraint).Hessian(x);
			vector<MatrixX> nonlinearIneq = (*_nonLinearIneqConstraint).Hessian(x);
			val.insert(val.end(), nonlinearIneq.begin(), nonlinearIneq.end());
			return val;
		}

		void BSplinePointToPointOptimization::nonLinearInequalityConstraint::loadConstraint(const socAssemblyPtr& socAssem, VectorU& optActiveJointIdx, unsigned int optActiveJointDOF, bool velConstraintExist, bool torqueConstraintExist, bool accConstraintExist)
		{
			_defaultState = socAssem->makeState();
			
			if (torqueConstraintExist)
			{
				_tauMax.resize(_defaultState->getDOF(State::TARGET_JOINT::ASSEMJOINT));
				_tauMin.resize(_defaultState->getDOF(State::TARGET_JOINT::ASSEMJOINT));
				int dof = 0;
				for (unsigned int i = 0; i < socAssem->getMateList().size(); i++)
				{
					_tauMax.block(dof, 0, socAssem->getJointPtrByMateIndex(i)->getDOF(), 1) = socAssem->getJointPtrByMateIndex(i)->getLimitInputUpper();
					_tauMin.block(dof, 0, socAssem->getJointPtrByMateIndex(i)->getDOF(), 1) = socAssem->getJointPtrByMateIndex(i)->getLimitInputLower();
					dof += socAssem->getJointPtrByMateIndex(i)->getDOF();
				}
			}

			_nConstraint = 0;
			if (velConstraintExist)
				_nConstraint += 2 * optActiveJointDOF;
			if (torqueConstraintExist)
				_nConstraint += 2 * _tauMax.size();
			if (accConstraintExist)
				_nConstraint += 2 * optActiveJointDOF;

			_velConstraintExist = velConstraintExist;
			_torqueConstraintExist = torqueConstraintExist;
			_accConstraintExist = accConstraintExist;

			_qdotMax.resize(optActiveJointDOF);
			_qdotMin.resize(optActiveJointDOF);
			_qddotMax.resize(optActiveJointDOF);
			_qddotMin.resize(optActiveJointDOF);
			unsigned int jointID;
			for (int i = 0, dof = 0; i < optActiveJointIdx.size(); i++)
			{
				jointID = _defaultState->getJointID(State::TARGET_JOINT::ACTIVEJOINT, optActiveJointIdx(i));
				for (int j = 0; j < socAssem->getJointPtrByMateIndex(jointID)->getDOF(); j++, dof++)
				{
					if (velConstraintExist)
					{
						_qdotMax(dof) = socAssem->getJointPtrByMateIndex(jointID)->getLimitVelUpper()(j);
						_qdotMin(dof) = socAssem->getJointPtrByMateIndex(jointID)->getLimitVelLower()(j);
					}
					if (accConstraintExist)
					{
						_qddotMax(dof) = socAssem->getJointPtrByMateIndex(jointID)->getLimitAccUpper()(j);
						_qddotMin(dof) = socAssem->getJointPtrByMateIndex(jointID)->getLimitAccLower()(j);
					}
				}
			}
		}

		VectorX BSplinePointToPointOptimization::nonLinearInequalityConstraint::func(const Math::VectorX & x) const
		{
			int nConstraint = 0;
			bool isTrajUpdated = false;
			VectorX val(_nConstraint);
			
			if (_torqueConstraintExist)
			{
				const MatrixX& tauTrj = _sharedDID->getTau(x);
				isTrajUpdated = true;
				for (int i = 0; i < tauTrj.rows(); i++)
				{
					val(nConstraint + i) = tauTrj.row(i).maxCoeff() - _tauMax(i);
					val(nConstraint + i + tauTrj.rows()) = _tauMin(i) - tauTrj.row(i).minCoeff();
				}
				nConstraint += 2 * tauTrj.rows();
			}
			
			if (_velConstraintExist)
			{
				if (!isTrajUpdated)
				{
					_sharedDID->compareControlPoint(x);
					isTrajUpdated = true;
				}
					
				const MatrixX& jointVelTrj = _sharedDID->getJointVel(_sharedDID->_optActiveJointIdx);
				for (int i = 0; i < _sharedDID->_optActiveJointDOF; i++)
				{
					val(nConstraint + i) = jointVelTrj.row(i).maxCoeff() - _qdotMax(i);
					val(nConstraint + _sharedDID->_optActiveJointDOF + i) = _qdotMin(i) - jointVelTrj.row(i).minCoeff();
				}
				nConstraint += 2 * _sharedDID->_optActiveJointDOF;
			}
			
			if (_accConstraintExist)
			{
				if (!isTrajUpdated) 
				{
					_sharedDID->compareControlPoint(x);
					isTrajUpdated = true;
				}
				const MatrixX& jointAccTrj = _sharedDID->getJointAcc(_sharedDID->_optActiveJointIdx);
				for (int i = 0; i < _sharedDID->_optActiveJointDOF; i++)
				{
					val(nConstraint + i) = jointAccTrj.row(i).maxCoeff() - _qddotMax(i);
					val(nConstraint + _sharedDID->_optActiveJointDOF + i) = _qddotMin(i) - jointAccTrj.row(i).minCoeff();
				}
				nConstraint += 2 * _sharedDID->_optActiveJointDOF;
			}

			return val;
		}

		MatrixX BSplinePointToPointOptimization::nonLinearInequalityConstraint::Jacobian(const Math::VectorX & x) const
		{
			MatrixX val(_nConstraint, x.size());
			int index;
			//int dofIdx;
			int mateID;
			bool isTrajUpdated = false;
			int nConstraint = 0;
			if (_torqueConstraintExist)
			{
				const MatrixX& tauTrj = _sharedDID->getTau(x);
				isTrajUpdated = true;
				const vector<MatrixX>& dtaudpTrj = _sharedDID->getdtaudp(x);
				for (int i = 0; i < tauTrj.rows(); i++)
				{
					tauTrj.row(i).maxCoeff(&index);
					val.row(nConstraint + i) = dtaudpTrj[index].row(i);
					tauTrj.row(i).minCoeff(&index);
					val.row(nConstraint + i + tauTrj.rows()) = -dtaudpTrj[index].row(i);
				}
				nConstraint += 2 * tauTrj.rows();
			}
			
			if (_velConstraintExist)
			{
				if (!isTrajUpdated)
				{
					_sharedDID->compareControlPoint(x);
					isTrajUpdated = true;
				}
				const MatrixX& jointVelTrj = _sharedDID->getJointVel(_sharedDID->_optActiveJointIdx);
				for (int i = 0, dofIdx = 0; i < _sharedDID->_optActiveJointIdx.size(); i++)
				{
					mateID = _sharedDID->_stateTrj[0]->getJointID(State::TARGET_JOINT::ACTIVEJOINT, _sharedDID->_optActiveJointIdx(i));
					for (unsigned int j = 0; j < _sharedDID->_socAssem->getJointPtrByMateIndex(mateID)->getDOF(); j++, dofIdx++)
					{
						jointVelTrj.row(dofIdx).maxCoeff(&index);
						val.row(nConstraint + dofIdx) = _sharedDID->_dqdotdp[index].row(_sharedDID->_stateTrj[0]->getAssemIndex(mateID) + j);
						jointVelTrj.row(dofIdx).minCoeff(&index);
						val.row(nConstraint + _sharedDID->_optActiveJointDOF + dofIdx) = -_sharedDID->_dqdotdp[index].row(_sharedDID->_stateTrj[0]->getAssemIndex(mateID) + j);
					}
				}
				nConstraint += 2 * _sharedDID->_optActiveJointDOF;
			}
			if (_accConstraintExist)
			{
				if (!isTrajUpdated)
				{
					_sharedDID->compareControlPoint(x);
					isTrajUpdated = true;
				}
				const MatrixX& jointAccTrj = _sharedDID->getJointAcc(_sharedDID->_optActiveJointIdx);
				for (int i = 0, dofIdx = 0; i < _sharedDID->_optActiveJointIdx.size(); i++)
				{
					mateID = _sharedDID->_stateTrj[0]->getJointID(State::TARGET_JOINT::ACTIVEJOINT, _sharedDID->_optActiveJointIdx(i));
					for (unsigned int j = 0; j < _sharedDID->_socAssem->getJointPtrByMateIndex(mateID)->getDOF(); j++, dofIdx++)
					{
						jointAccTrj.row(dofIdx).maxCoeff(&index);
						val.row(nConstraint + dofIdx) = _sharedDID->_dqddotdp[index].row(_sharedDID->_stateTrj[0]->getAssemIndex(mateID) + j);
						jointAccTrj.row(dofIdx).minCoeff(&index);
						val.row(nConstraint + _sharedDID->_optActiveJointDOF + dofIdx) = -_sharedDID->_dqddotdp[index].row(_sharedDID->_stateTrj[0]->getAssemIndex(mateID) + j);
					}
				}
				nConstraint += 2 * _sharedDID->_optActiveJointDOF;
			}

			return val;
		}
		vector<MatrixX> BSplinePointToPointOptimization::nonLinearInequalityConstraint::Hessian(const Math::VectorX & x) const
		{
			vector<MatrixX> val(_nConstraint, MatrixX::Zero(x.size(), x.size()));
			
			if (_torqueConstraintExist)
			{
				const MatrixX& tauTrj = _sharedDID->getTau(x);
				const vector<vector<MatrixX>>& dtau2dp2Trj = _sharedDID->getd2taudp2(x);
				int index;
				for (int i = 0; i < tauTrj.rows(); i++)
				{
					tauTrj.row(i).maxCoeff(&index);
					val[i] = dtau2dp2Trj[index][i];
					tauTrj.row(i).minCoeff(&index);
					val[i + tauTrj.rows()] = -dtau2dp2Trj[index][i];
				}
			}

			return val;
		}

		void BSplinePointToPointOptimization::nonLinearInequalitySmallConstraint::loadConstraint(const socAssemblyPtr& socAssem)
		{
			StatePtr defaultState = socAssem->makeState();

			_tauMax.resize(defaultState->getDOF(State::TARGET_JOINT::ASSEMJOINT));
			_tauMin.resize(defaultState->getDOF(State::TARGET_JOINT::ASSEMJOINT));
			int dof = 0;
			for (unsigned int i = 0; i < socAssem->getMateList().size(); i++)
			{
				_tauMax.block(dof, 0, socAssem->getJointPtrByMateIndex(i)->getDOF(), 1) = socAssem->getJointPtrByMateIndex(i)->getLimitInputUpper();
				_tauMin.block(dof, 0, socAssem->getJointPtrByMateIndex(i)->getDOF(), 1) = socAssem->getJointPtrByMateIndex(i)->getLimitInputLower();
				dof += socAssem->getJointPtrByMateIndex(i)->getDOF();
			}
		}

		VectorX BSplinePointToPointOptimization::nonLinearInequalitySmallConstraint::func(const Math::VectorX & x) const
		{
			const MatrixX& tauTrj = _sharedDID->getTau(x);
			VectorX val(2 * tauTrj.rows());

			for (int i = 0; i < tauTrj.rows(); i++)
			{
				val(i) = tauTrj.row(i).maxCoeff() - _tauMax(i);
				val(i + tauTrj.rows()) = _tauMin(i) - tauTrj.row(i).minCoeff();
			}

			return val;
		}

		Math::MatrixX BSplinePointToPointOptimization::nonLinearInequalitySmallConstraint::Jacobian(const Math::VectorX & x) const
		{
			const MatrixX& tauTrj = _sharedDID->getTau(x);
			const vector<MatrixX>& dtaudpTrj = _sharedDID->getdtaudp(x);

			Math::MatrixX val(2 * tauTrj.rows(), x.size());
			int index;
			for (int i = 0; i < tauTrj.rows(); i++)
			{
				tauTrj.row(i).maxCoeff(&index);
				val.row(i) = dtaudpTrj[index].row(i);
				tauTrj.row(i).minCoeff(&index);
				val.row(i + tauTrj.rows()) = -dtaudpTrj[index].row(i);
			}

			return val;
		}
		vector<MatrixX> BSplinePointToPointOptimization::nonLinearInequalitySmallConstraint::Hessian(const Math::VectorX & x) const
		{
			const MatrixX& tauTrj = _sharedDID->getTau(x);
			const vector<vector<MatrixX>>& dtau2dp2Trj = _sharedDID->getd2taudp2(x);
			const MatrixX& jointVelTrj = _sharedDID->getJointVel(_sharedDID->_optActiveJointIdx);
			vector<MatrixX> val(2 * tauTrj.rows(), MatrixX::Zero(x.size(), x.size()));
			int index;

			for (int i = 0; i < tauTrj.rows(); i++)
			{
				tauTrj.row(i).maxCoeff(&index);
				val[i] = dtau2dp2Trj[index][i];
				tauTrj.row(i).minCoeff(&index);
				val[i + tauTrj.rows()] = -dtau2dp2Trj[index][i];
			}

			return val;
		}

		///////////////////////////////////////// test function ////////////////////////////////////

		Math::VectorX BSplinePointToPointOptimization::energyLossTestFunction::func(const Math::VectorX & x) const
		{
			const MatrixX& tauTrj = _sharedDID->getTau(x);
			shared_ptr<MotorJoint> tempJointPtr;
			Real voltage;
			Real current;
			VectorX val(1);
			val(0) = 0;
			for (int i = 0; i < tauTrj.cols(); i++)
			{
				for (unsigned int j = 0, dofIdx = 0; j < _sharedDID->_socAssem->getMateList().size(); j++)
				{
					tempJointPtr = static_pointer_cast<MotorJoint>(_sharedDID->_socAssem->getJointPtrByMateIndex(j));
					for (unsigned int k = 0; k < tempJointPtr->getDOF(); k++, dofIdx++)
					{
						current = 1.0 / (tempJointPtr->getMotorConstant() * tempJointPtr->getGearRatio()) * tauTrj(dofIdx, i)
							+ tempJointPtr->getRotorInertia() * tempJointPtr->getGearRatio() / tempJointPtr->getMotorConstant() * _sharedDID->_stateTrj[i]->getJointStateByMateIndex(j).getqddot()(k);
						voltage = current * tempJointPtr->getResistance() + tempJointPtr->getBackEMFConstant() * tempJointPtr->getGearRatio() * _sharedDID->_stateTrj[i]->getJointStateByMateIndex(j).getqdot()(k);
						val(0) += _sharedDID->_timeSpanWeight(i) * max(current * voltage, 0.0);

					}
				}
			}
			return val;
		}


		Math::VectorX BSplinePointToPointOptimization::trajectoryCheck::func(const Math::VectorX & x) const
		{
			_sharedDID->compareControlPoint(x);
			return _sharedDID->getJointVel(_activeJointIdx).col(_timeStep);
		}


		void BSplinePointToPointOptimization::nonLinearInequalityTestConstraint::loadConstraint(const socAssemblyPtr& socAssem, VectorU& optActiveJointIdx, unsigned int optActiveJointDOF, bool velConstraintExist, bool torqueConstraintExist, bool accConstraintExist)
		{
			_defaultState = socAssem->makeState();

			if (torqueConstraintExist)
			{
				_tauMax.resize(_defaultState->getDOF(State::TARGET_JOINT::ASSEMJOINT));
				_tauMin.resize(_defaultState->getDOF(State::TARGET_JOINT::ASSEMJOINT));
				int dof = 0;
				for (unsigned int i = 0; i < socAssem->getMateList().size(); i++)
				{
					_tauMax.block(dof, 0, socAssem->getJointPtrByMateIndex(i)->getDOF(), 1) = socAssem->getJointPtrByMateIndex(i)->getLimitInputUpper();
					_tauMin.block(dof, 0, socAssem->getJointPtrByMateIndex(i)->getDOF(), 1) = socAssem->getJointPtrByMateIndex(i)->getLimitInputLower();
					dof += socAssem->getJointPtrByMateIndex(i)->getDOF();
				}
			}

			_nConstraint = 0;
			if (velConstraintExist)
				_nConstraint += 2 * optActiveJointDOF;
			if (torqueConstraintExist)
				_nConstraint += 2 * _tauMax.size();
			if (accConstraintExist)
				_nConstraint += 2 * optActiveJointDOF;

			_velConstraintExist = velConstraintExist;
			_torqueConstraintExist = torqueConstraintExist;
			_accConstraintExist = accConstraintExist;

			_qdotMax.resize(optActiveJointDOF);
			_qdotMin.resize(optActiveJointDOF);
			_qddotMax.resize(optActiveJointDOF);
			_qddotMin.resize(optActiveJointDOF);
			unsigned int jointID;
			for (int i = 0, dof = 0; i < optActiveJointIdx.size(); i++)
			{
				jointID = _defaultState->getJointID(State::TARGET_JOINT::ACTIVEJOINT, optActiveJointIdx(i));
				for (unsigned int j = 0; j < socAssem->getJointPtrByMateIndex(jointID)->getDOF(); j++, dof++)
				{
					if (velConstraintExist)
					{
						_qdotMax(dof) = socAssem->getJointPtrByMateIndex(jointID)->getLimitVelUpper()(j);
						_qdotMin(dof) = socAssem->getJointPtrByMateIndex(jointID)->getLimitVelLower()(j);
					}
					if (accConstraintExist)
					{
						_qddotMax(dof) = socAssem->getJointPtrByMateIndex(jointID)->getLimitAccUpper()(j);
						_qddotMin(dof) = socAssem->getJointPtrByMateIndex(jointID)->getLimitAccLower()(j);
					}
				}
			}
		}

		VectorX BSplinePointToPointOptimization::nonLinearInequalityTestConstraint::func(const Math::VectorX & x) const
		{
			int nConstraint = 0;
			bool isTrajUpdated = false;
			VectorX val(_nConstraint);

			if (_torqueConstraintExist)
			{
				const MatrixX& tauTrj = _sharedDID->getTau(x);
				isTrajUpdated = true;
				for (int i = 0; i < tauTrj.rows(); i++)
				{
					val(nConstraint + i) = tauTrj.row(i).maxCoeff() - _tauMax(i);
					val(nConstraint + i + tauTrj.rows()) = _tauMin(i) - tauTrj.row(i).minCoeff();
				}
				nConstraint += 2 * tauTrj.rows();
			}

			if (_velConstraintExist)
			{
				if (!isTrajUpdated)
				{
					_sharedDID->compareControlPoint(x);
					isTrajUpdated = true;
				}

				const MatrixX& jointVelTrj = _sharedDID->getJointVel(_sharedDID->_optActiveJointIdx);
				for (int i = 0; i < _sharedDID->_optActiveJointDOF; i++)
				{
					val(nConstraint + i) = jointVelTrj.row(i).maxCoeff() - _qdotMax(i);
					val(nConstraint + _sharedDID->_optActiveJointDOF + i) = _qdotMin(i) - jointVelTrj.row(i).minCoeff();
				}
				nConstraint += 2 * _sharedDID->_optActiveJointDOF;
			}

			if (_accConstraintExist)
			{
				if (!isTrajUpdated)
				{
					_sharedDID->compareControlPoint(x);
					isTrajUpdated = true;
				}
				const MatrixX& jointAccTrj = _sharedDID->getJointAcc(_sharedDID->_optActiveJointIdx);
				for (int i = 0; i < _sharedDID->_optActiveJointDOF; i++)
				{
					val(nConstraint + i) = jointAccTrj.row(i).maxCoeff() - _qddotMax(i);
					val(nConstraint + _sharedDID->_optActiveJointDOF + i) = _qddotMin(i) - jointAccTrj.row(i).minCoeff();
				}
				nConstraint += 2 * _sharedDID->_optActiveJointDOF;
			}

			return val;
		}




	}
}

