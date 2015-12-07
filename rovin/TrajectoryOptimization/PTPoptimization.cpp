#include "PTPoptimization.h"
#include <rovin/Model/MotorJoint.h>

using namespace std;
using namespace rovin::Math;
using namespace rovin::Model;

namespace rovin
{
	namespace TrajectoryOptimization
	{



		//Real effortFunction(const vector<State*>& stateTrj)
		//{
		//	unsigned int nStep = stateTrj.size();
		//	Real effort = 0.0;

		//	for (int i = 0; i < nStep; i++)
		//		effort += (stateTrj[i]->getJointTorque(State::TARGET_JOINT::ACTIVEJOINT)).squaredNorm();

		//	return effort;
		//}

		//Real energyLossFunction(const vector<State*>& stateTrj)
		//{
		//	unsigned int nStep = stateTrj.size();
		//	unsigned int nDOF = stateTrj[0]->getActiveJointDof();
		//	Real energyLoss = 0.0;
		//	VectorX torque(nDOF);
		//	for (int i = 0; i < nStep; i++)
		//	{
		//		torque = stateTrj[i]->getJointTorque(State::TARGET_JOINT::ACTIVEJOINT);
		//		for (int j = 0; j < nDOF; j++)
		//			energyLoss += max(torque(j, 0) * stateTrj[i]->getJointqdot(j)(0, 0), 0.0);
		//	}

		//	return energyLoss;
		//}

		//Real energyExchangeLossFunction(const vector<State*>& stateTrj)
		//{
		//	unsigned int nStep = stateTrj.size();
		//	Real energyLoss = 0.0;
		//	for (int i = 0; i < nStep; i++)
		//		energyLoss += max((stateTrj[i]->getJointTorque(State::TARGET_JOINT::ACTIVEJOINT).dot(stateTrj[i]->getJointqdot(State::TARGET_JOINT::ACTIVEJOINT))), 0.0);

		//	return energyLoss;
		//}

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
			_nDOF = _defaultState->getActiveJointDof();
		}

		void PointToPointOptimization::setFinalTimeAndTimeStep(const Math::Real tf, const int nStep)
		{
			_tf = tf;
			_nStep = nStep;
			_dt = tf / (Real)(nStep - 1.0);
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

		void PointToPointOptimization::setOptimizingJointIndex(const vector<unsigned int>& optJointIdx)
		{
			_optActiveJointIdx = optJointIdx;
			int nJoint = _socAssem->getMateList().size();
			vector<unsigned int> noptJointIdx(0);
			int j = 0;
			int optJointDOF = 0;
			int noptJointDOF = 0;
			int k = nJoint;
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
					noptJointIdx.push_back(i);
					noptJointDOF += _defaultState->getJointState(i)._dof;
				}
			}
			for (int i = k; i < nJoint; i++)
			{
				noptJointIdx.push_back(i);
				noptJointDOF += _defaultState->getJointState(i)._dof;
			}
			_optActiveJointDOF = optJointDOF;
			_noptActiveJointDOF = noptJointDOF;
			_noptActiveJointIdx = noptJointIdx;
		}

		void PointToPointOptimization::setConstraintRange(bool posConstraintExist, bool velConstraintExist, bool accConstraintExist, bool jerkConstraintExist)
		{
			_posConstraintExist = posConstraintExist;
			_velConstraintExist = velConstraintExist;
			_accConstraintExist = accConstraintExist;
			_jerkConstraintExist = jerkConstraintExist;
		}


		BSplinePointToPointOptimization::SharedDID::SharedDID(const socAssemblyPtr socAssem, const int nStep, const Real dt, const VectorX & knot, const VectorX & noptControlPoint)
			: _socAssem(socAssem), _nStep(nStep), _dt(dt), _knot(knot)
		{
			_isInitiated = false;

			_noptJointValSpline = Math::BSpline<-1, -1, -1>(knot, noptControlPoint);
			_noptJointVelSpline = _noptJointValSpline.derivative();
			_noptJointAccSpline = _noptJointVelSpline.derivative();
			_noptJointJerkSpline = _noptJointAccSpline.derivative();

			_stateTrj = vector< StatePtr >(nStep);
			for (unsigned int i = 0; i < _stateTrj.size(); i++)
			{
				_stateTrj[i] = socAssem->makeState();
				//_stateTrj[i]->setJointq(ACTIVEJOINT, _noptActiveJointIdx, _noptJointValSpline(_dt*i));
				//_stateTrj[i]->setJointq(ACTIVEJOINT, _noptActiveJointIdx, _noptJointValSpline(_dt*i));
				//_stateTrj[i]->setJointq(ACTIVEJOINT, _noptActiveJointIdx, _noptJointValSpline(_dt*i));
			}

			// calculate dqdp
		}

		const MatrixX& BSplinePointToPointOptimization::SharedDID::getTau(const VectorX & controlPoint)
		{
			compareControlPoint(controlPoint);

			return _tau;
		}

		const vector<MatrixX>& BSplinePointToPointOptimization::SharedDID::getdTaudp(const Math::VectorX & controlPoint)
		{
			compareControlPoint(controlPoint);

			if (!_isDIDUpdated)
			{
				_dtaudp = vector< MatrixX >(_stateTrj.size());
				_d2taudp2 = vector< vector< MatrixX >>(_stateTrj.size());

				pair< MatrixX, std::vector< MatrixX >> DID;
				for (unsigned int i = 0; i < _stateTrj.size(); i++)
				{
					//DID = Dynamics::differentiateInverseDynamics(*_socAssem, *_stateTrj[i], _dqdp[i], _dqdotdp[i], _dqddotdp[i]);
					_dtaudp[i] = DID.first;
					_d2taudp2[i] = DID.second;
				}

				_isDIDUpdated = true;
			}

			return _dtaudp;
		}

		const vector< vector< MatrixX >>& BSplinePointToPointOptimization::SharedDID::getd2Taudp2(const Math::VectorX & controlPoint)
		{
			compareControlPoint(controlPoint);

			if (!_isDIDUpdated)
			{
				_dtaudp = vector< MatrixX >(_stateTrj.size());
				_d2taudp2 = vector< vector< MatrixX >>(_stateTrj.size());

				pair< MatrixX, std::vector< MatrixX >> DID;
				for (unsigned int i = 0; i < _stateTrj.size(); i++)
				{
					//DID = Dynamics::differentiateInverseDynamics(*_socAssem, *_stateTrj[i], _dqdp[i], _dqdotdp[i], _dqddotdp[i]);
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

				_optJointValSpline = Math::BSpline<-1, -1, -1>(_knot, _currentControlPoint);
				_optJointVelSpline = _optJointValSpline.derivative();
				_optJointAccSpline = _optJointVelSpline.derivative();
				_optJointJerkSpline = _optJointAccSpline.derivative();

				for (unsigned int i = 0; i < _stateTrj.size(); i++)
				{
					//_stateTrj[i]->setJointq(ACTIVEJOINT, _optActiveJointIdx, _optJointValSpline(_dt*i));
					//_stateTrj[i]->setJointq(ACTIVEJOINT, _optActiveJointIdx, _optJointValSpline(_dt*i));
					//_stateTrj[i]->setJointq(ACTIVEJOINT, _optActiveJointIdx, _optJointValSpline(_dt*i));
					Dynamics::solveInverseDynamics(*_socAssem, *_stateTrj[i]);
				}

				_tau = MatrixX(_stateTrj[0]->getTotalJointDof(),_stateTrj.size());
				for (unsigned int i = 0; i < _stateTrj.size(); i++)
				{
					// tau[i] = _stateTrj[i]->getActiveJointTau();
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

		void BSplinePointToPointOptimization::run(const ObjectiveFunctionType & objectiveType)
		{

			//std::shared_ptr<SharedDID> sharedDID = std::shared_ptr<SharedDID>(new SharedDID(_socAssem, _nStep, _dt, _knot, _knot);
			if (objectiveType == ObjectiveFunctionType::Effort)
			{
				_objectiveFunc = Math::FunctionPtr(new effortFunction());
				//std::static_pointer_cast<effortFunction>(_objectiveFunc)->_sharedDID = sharedDID;
			}

			///////////////////////////////////////// EQUALITY CONSTRAINT ///////////////////////////////////
			_eqFunc = Math::FunctionPtr(new LinearFunction());
			static_pointer_cast<LinearFunction>(_eqFunc)->A = Aeq_opt;
			static_pointer_cast<LinearFunction>(_eqFunc)->b = beq_opt;

			/////////////////////////////////////////////////////////////////////////////////////////////////

			/////////////////////////////////////// INEQUALITY CONSTRAINT ///////////////////////////////////
			_ineqFunc = Math::FunctionPtr(new inequalityConstraint());
			std::shared_ptr<LinearFunction> linearIneqFunc = std::shared_ptr<LinearFunction>(new LinearFunction());
			linearIneqFunc->A = Aineq_opt;
			linearIneqFunc->b = bineq_opt;
			static_pointer_cast<inequalityConstraint>(_ineqFunc)->_linearIneqConstraint = linearIneqFunc;

			std::shared_ptr<nonLinearInequalityConstraint> nonLinearIneqFunc = std::shared_ptr<nonLinearInequalityConstraint>(new nonLinearInequalityConstraint());
			
			nonLinearIneqFunc->_tauMax.resize(_defaultState->getTotalJointDof());
			nonLinearIneqFunc->_tauMin.resize(_defaultState->getTotalJointDof());
			int dof = 0;
			for (unsigned int i = 0; i < _socAssem->getMateList().size(); i++)
			{
				//nonLinearIneqFunc->_tauMax.block(dof, 0, _socAssem->getJointPtrByMateIndex(i)->getDOF(), 1) = _socAssem->getJointPtrByMateIndex(i)->getLimitTorqueUpper();
				//nonLinearIneqFunc->_tauMin.block(dof, 0, _socAssem->getJointPtrByMateIndex(i)->getDOF(), 1) = _socAssem->getJointPtrByMateIndex(i)->getLimitTorqueLower();
				dof += _socAssem->getJointPtrByMateIndex(i)->getDOF();
			}

			/////////////////////////////////////////////////////////////////////////////////////////////////
		}

		void BSplinePointToPointOptimization::setSplineCondition(const int order, const int nMiddleCP)
		{
			assert(nMiddleCP >= _waypoint.size() && "The number of control points should be bigger than the number of waypoints");
			_order = order;
			_nMiddleCP = nMiddleCP;
			int nK = order + nMiddleCP;
			int nInit = 0;
			int nFinal = 0;
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
			Real minT = _tf;
			Real knotStep;

			for (int i = 0; i < order; i++)
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

					for (int i = 0; i < nInit - 1; i++)
						knot(order + i) = knot(order + nInit - 1) * (Real)(i + 1) / (Real)nInit;

					for (int i = 0; i < nFinal - 1; i++)
						knot(nK - order - 1 - i) = _tf - (_tf - knot(nK - order - nFinal)) * (Real)(i + 1) / (Real)nFinal;
				}
				else
				{
					////////////////////////////
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

		std::pair<Math::MatrixX, Math::VectorX> BSplinePointToPointOptimization::generateLinearEqualityConstraint(const std::vector<unsigned int>& activeJointIdx, const int activeJointDOF)
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
			Aeq_opt.resize(_optActiveJointDOF*_waypoint.size(), _nMiddleCP*_optActiveJointDOF);
			beq_opt.resize(_optActiveJointDOF*_waypoint.size());
			Aeq_opt.setZero();
			beq_opt.setZero();
			Aeq_nopt.resize(_noptActiveJointDOF*_waypoint.size(), _nMiddleCP*_noptActiveJointDOF);
			beq_nopt.resize(_noptActiveJointDOF*_waypoint.size());
			Aeq_nopt.setZero();
			beq_nopt.setZero();
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
					Aeq_opt.block(i*_optActiveJointDOF + j, j*_nMiddleCP, 1, _nMiddleCP) = Ni.block(0, _nInitCP, 1, _nMiddleCP);
					beq_opt(i*_optActiveJointDOF + j) = -_waypoint[i].first(_optActiveJointIdx[j]);
					for (int k = 0; k < _nInitCP; k++)
						beq_opt(i*_optActiveJointDOF + j) += Ni(0, k)*BoundaryCP[k](_optActiveJointIdx[j]);
					for (int k = 0; k < _nFinalCP; k++)
						beq_opt(i*_optActiveJointDOF + j) += Ni(0, _nInitCP + _nMiddleCP + _nFinalCP - 1 - k)*BoundaryCP[5 - k](_optActiveJointIdx[j]);
				}
				for (int j = 0; j < _noptActiveJointDOF; j++)
				{
					Aeq_nopt.block(i*_noptActiveJointDOF + j, j*_nMiddleCP, 1, _nMiddleCP) = Ni.block(0, _nInitCP, 1, _nMiddleCP);
					beq_nopt(i*_noptActiveJointDOF + j) = -_waypoint[i].first(_noptActiveJointIdx[j]);
					for (int k = 0; k < _nInitCP; k++)
						beq_nopt(i*_noptActiveJointDOF + j) += Ni(0, k)*BoundaryCP[k](_noptActiveJointIdx[j]);
					for (int k = 0; k < _nFinalCP; k++)
						beq_nopt(i*_noptActiveJointDOF + j) += Ni(0, _nInitCP + _nMiddleCP + _nFinalCP - 1 - k)*BoundaryCP[5 - k](_noptActiveJointIdx[j]);
				}
			}
		}

		pair<MatrixX, VectorX> BSplinePointToPointOptimization::generateLinearInequalityConstraint(const vector<unsigned int>& activeJointIdx, const int activeJointDOF)
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

			if (_jerkConstraintExist)
				nConstraint += 2;
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
				for (unsigned int l = 0; l < activeJointIdx.size(); l++)
				{
					tempJointPtr = _socAssem->getJointPtr(_defaultState->getActiveJointList()[activeJointIdx[l]]);
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
					ti = (Real)i * _dt;
					for (int j = 0; j < _nMiddleCP + _nInitCP + _nFinalCP; j++)
					{
						tempControlPoint(j) = 1.0;
						BSpline<-1, -1, -1> tempSpline(_knot, tempControlPoint);
						tempControlPoint(j) = 0.0;
						BSpline<-1, -1, -1> dtempSpline = tempSpline.derivative();
						dNi(j) = dtempSpline(ti)(0);
						BSpline<-1, -1, -1> ddtempSpline = dtempSpline.derivative();
						ddNi(j) = ddtempSpline(ti)(0);
						if (_jerkConstraintExist)
						{
							BSpline<-1, -1, -1> dddtempSpline = ddtempSpline.derivative();
							dddNi(j) = dddtempSpline(ti)(0);
						}
					}

					// add velocity constraint
					if (_velConstraintExist)
					{
						for (int j = 0; j < activeJointDOF; j++)
						{
							Aineq.block(nStartVel + activeJointDOF*i + j, _nMiddleCP*j, 1, _nMiddleCP) = dNi.block(0, _nInitCP, 1, _nMiddleCP);		// less than maximum
							Aineq.block(nStartVel + activeJointDOF * _nStep + activeJointDOF*i + j, _nMiddleCP*j, 1, _nMiddleCP) = -dNi.block(0, _nInitCP, 1, _nMiddleCP);   // bigger than minimum
							for (int k = 0; k < _nInitCP; k++)
							{
								temp = dNi(k)*BoundaryCP[k](activeJointIdx[j]);
								bineq(nStartVel + activeJointDOF*i + j) += temp;
								bineq(nStartVel + activeJointDOF * _nStep + activeJointDOF*i + j) -= temp;
							}
							for (int k = 0; k < _nFinalCP; k++)
							{
								temp = dNi(_nInitCP + _nMiddleCP + _nFinalCP - 1 - k)*BoundaryCP[k](activeJointIdx[j]);
								bineq(nStartVel + activeJointDOF*i + j) += temp;
								bineq(nStartVel + activeJointDOF * _nStep + activeJointDOF*i + j) -= temp;
							}
						}
					}
					// add acceleration constraint
					if (_accConstraintExist)
					{
						for (int j = 0; j < activeJointDOF; j++)
						{
							Aineq.block(nStartAcc + activeJointDOF*i + j, _nMiddleCP*j, 1, _nMiddleCP) = ddNi.block(0, _nInitCP, 1, _nMiddleCP);		// less than maximum
							Aineq.block(nStartAcc + activeJointDOF * _nStep + activeJointDOF*i + j, _nMiddleCP*j, 1, _nMiddleCP) = -ddNi.block(0, _nInitCP, 1, _nMiddleCP);   // bigger than minimum
																																													  //bineq(nStartVel + activeJointDOF*i + j) = getVelocityLimit;
							for (int k = 0; k < _nInitCP; k++)
							{
								temp = ddNi(k)*BoundaryCP[k](activeJointIdx[j]);
								bineq(nStartAcc + activeJointDOF*i + j) += temp;
								bineq(nStartAcc + activeJointDOF * _nStep + activeJointDOF*i + j) -= temp;
							}
							for (int k = 0; k < _nFinalCP; k++)
							{
								temp = ddNi(_nInitCP + _nMiddleCP + _nFinalCP - 1 - k)*BoundaryCP[k](activeJointIdx[j]);
								bineq(nStartAcc + activeJointDOF*i + j) += temp;
								bineq(nStartAcc + activeJointDOF * _nStep + activeJointDOF*i + j) -= temp;
							}
						}
					}
					// add jerk constraint
					if (_jerkConstraintExist)
					{
						for (int j = 0; j < activeJointDOF; j++)
						{
							Aineq.block(nStartJerk + activeJointDOF*i + j, _nMiddleCP*j, 1, _nMiddleCP) = dddNi.block(0, _nInitCP, 1, _nMiddleCP);		// less than maximum
							Aineq.block(nStartJerk + activeJointDOF * _nStep + activeJointDOF*i + j, _nMiddleCP*j, 1, _nMiddleCP) = -dddNi.block(0, _nInitCP, 1, _nMiddleCP);   // bigger than minimum
																																										  //bineq(nStartVel + activeJointDOF*i + j) = getVelocityLimit;
							for (int k = 0; k < _nInitCP; k++)
							{
								temp = dddNi(k)*BoundaryCP[k](activeJointIdx[j]);
								bineq(nStartJerk + activeJointDOF*i + j) += temp;
								bineq(nStartJerk + activeJointDOF * _nStep + activeJointDOF*i + j) -= temp;
							}
							for (int k = 0; k < _nFinalCP; k++)
							{
								temp = dddNi(_nInitCP + _nMiddleCP + _nFinalCP - 1 - k)*BoundaryCP[k](activeJointIdx[j]);
								bineq(nStartJerk + activeJointDOF*i + j) += temp;
								bineq(nStartJerk + activeJointDOF * _nStep + activeJointDOF*i + j) -= temp;
							}
						}
					}
				}

				if (_velConstraintExist)
				{
					joint_l = 0;
					for (unsigned int l = 0; l < activeJointIdx.size(); l++)
					{
						tempJointPtr = _socAssem->getJointPtr(_defaultState->getActiveJointList()[activeJointIdx[l]]);
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
					for (unsigned int l = 0; l < activeJointIdx.size(); l++)
					{
						tempJointPtr = _socAssem->getJointPtr(_defaultState->getActiveJointList()[activeJointIdx[l]]);
						tempUpperLimit = tempJointPtr->getLimitAccupper();
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

				if (_jerkConstraintExist)
				{
					joint_l = 0;
					for (unsigned int l = 0; l < activeJointIdx.size(); l++)
					{
						tempJointPtr = _socAssem->getJointPtr(_defaultState->getActiveJointList()[activeJointIdx[l]]);
						//////////////////////////////////////////////////////////////// change to jerk
						tempUpperLimit = tempJointPtr->getLimitVelUpper();
						tempLowerLimit = tempJointPtr->getLimitVelLower();
						////////////////////////////////////////////////////////////////
						for (unsigned int j = 0; j < tempJointPtr->getDOF(); j++)
						{
							for (int i = 0; i < _nStep; i++)
							{
								bineq(nStartJerk + activeJointDOF*i + joint_l + j) -= tempUpperLimit(j);
								bineq(nStartJerk + activeJointDOF * _nStep + activeJointDOF*i + joint_l + j) += tempLowerLimit(j);
							}
						}
						joint_l += tempJointPtr->getDOF();
					}
				}

			}
			Ineq.first = Aineq;
			Ineq.second = bineq;
			return Ineq;
		}

		void BSplinePointToPointOptimization::generateLinearInequalityConstraint()
		{
			pair<MatrixX, VectorX> Ineq = generateLinearInequalityConstraint(_optActiveJointIdx, _optActiveJointDOF);
			Aineq_opt = Ineq.first;
			bineq_opt = Ineq.second;
			Ineq = generateLinearInequalityConstraint(_noptActiveJointIdx, _noptActiveJointDOF);
			Aineq_nopt = Ineq.first;
			bineq_nopt = Ineq.second;
		}

		void BSplinePointToPointOptimization::generateNoptControlPoint()
		{


		}

		Math::VectorX BSplinePointToPointOptimization::effortFunction::func(const Math::VectorX & x) const
		{
			const MatrixX& tauTrj = _sharedDID->getTau(x);

			VectorX val(1);
			val(0) = tauTrj.squaredNorm()*_sharedDID->_dt;
			return val;
		}

		Math::MatrixX BSplinePointToPointOptimization::effortFunction::Jacobian(const Math::VectorX & x) const
		{
			const MatrixX& tauTrj = _sharedDID->getTau(x);

			const vector<MatrixX>& dtaudpTrj = _sharedDID->getdTaudp(x);
			Math::MatrixX val(1, x.size());
			for (int i = 0; i < tauTrj.cols(); i++)
			{
				val += tauTrj.col(i).transpose()*dtaudpTrj[i];
			}

			return 2.0*_sharedDID->_dt*val;
		}

		vector<Math::MatrixX> BSplinePointToPointOptimization::effortFunction::Hessian(const Math::VectorX & x) const
		{
			const MatrixX& tauTrj = _sharedDID->getTau(x);
			const vector<MatrixX>& dtaudpTrj = _sharedDID->getdTaudp(x);
			const vector<vector<MatrixX>>& d2taudp2Trj = _sharedDID->getd2Taudp2(x);
			vector<Math::MatrixX> val(1, MatrixX::Zero(x.size(),x.size()));
			for (int i = 0; i < tauTrj.cols(); i++)
			{
				for (int j = 0; j < tauTrj.rows(); j++)
					val[0] += 2.0 * _sharedDID->_dt * tauTrj(j,i)*d2taudp2Trj[i][j];
				val[0] += 2.0 * _sharedDID->_dt * dtaudpTrj[i].transpose()*dtaudpTrj[i];
			}
			
			return val;
		}

		Math::VectorX BSplinePointToPointOptimization::energyLossFunction::func(const Math::VectorX & x) const
		{
			const MatrixX& tauTrj = _sharedDID->getTau(x);
			shared_ptr<MotorJoint> tempJointPtr;
			Real voltage;
			Real current;
			VectorX val(1);
			for (int i = 0; i < tauTrj.cols(); i++)
			{
				for (int j = 0, dofIdx = 0; j < _sharedDID->_socAssem->getMateList().size(); j++)
				{
					tempJointPtr = static_pointer_cast<MotorJoint>(_sharedDID->_socAssem->getJointPtrByMateIndex(j));
					for (int k = 0; k < tempJointPtr->getDOF(); k++, dofIdx++)
					{
						current = 1.0 / (tempJointPtr->getMotorConstant() * tempJointPtr->getGearRatio()) * tauTrj(dofIdx, i)
							+ 1.0 / tempJointPtr->getMotorConstant() * tempJointPtr->getRotorInertia()*_sharedDID->_stateTrj[i]->getJointStateByMateIndex(j).getqddot()(k);
						voltage = current * tempJointPtr->getResistance() + tempJointPtr->getBackEMFConstant() * tempJointPtr->getGearRatio() * _sharedDID->_stateTrj[i]->getJointStateByMateIndex(j).getqdot()(k);
						val(0) += max(current * voltage, 0.0);
						
					}
				}
			}
			return _sharedDID->_dt * val;
		}

		Math::MatrixX BSplinePointToPointOptimization::energyLossFunction::Jacobian(const Math::VectorX & x) const
		{
			const MatrixX& tauTrj = _sharedDID->getTau(x);
			const vector<MatrixX>& dtaudpTrj = _sharedDID->getdTaudp(x);
			shared_ptr<MotorJoint> tempJointPtr;
			Real voltage;
			Real current;
			Math::MatrixX val(1, x.size());
			val.setZero();
			Math::MatrixX dcurrentdp(1, x.size());
			for (int i = 0; i < tauTrj.cols(); i++)
			{
				for (int j = 0, dofIdx = 0; j < _sharedDID->_socAssem->getMateList().size(); j++)
				{
					tempJointPtr = static_pointer_cast<MotorJoint>(_sharedDID->_socAssem->getJointPtrByMateIndex(j));
					for (int k = 0; k < tempJointPtr->getDOF(); k++, dofIdx++)
					{
						current = 1.0 / (tempJointPtr->getMotorConstant() * tempJointPtr->getGearRatio()) * tauTrj(dofIdx, i)
							+ tempJointPtr->getRotorInertia() / tempJointPtr->getMotorConstant() * _sharedDID->_stateTrj[i]->getJointStateByMateIndex(j).getqddot()(k);
						voltage = current * tempJointPtr->getResistance() + tempJointPtr->getBackEMFConstant() * tempJointPtr->getGearRatio() * _sharedDID->_stateTrj[i]->getJointStateByMateIndex(j).getqdot()(k);
						
						if (current * voltage > 0.0)
						{
							dcurrentdp = 1.0 / (tempJointPtr->getMotorConstant() * tempJointPtr->getGearRatio()) * dtaudpTrj[i].row(dofIdx)
								+ tempJointPtr->getRotorInertia() / tempJointPtr->getMotorConstant() * _sharedDID->_dqdp[i].row(dofIdx);
							val += (current * tempJointPtr->getResistance() + voltage) * dcurrentdp 
								+ current * tempJointPtr->getBackEMFConstant()*tempJointPtr->getGearRatio() * _sharedDID->_dqdotdp[i].row(dofIdx);
						}

					}
				}
			}

			return _sharedDID->_dt * val;
		}

		vector<Math::MatrixX> BSplinePointToPointOptimization::energyLossFunction::Hessian(const Math::VectorX & x) const
		{

			const MatrixX& tauTrj = _sharedDID->getTau(x);
			const vector<MatrixX>& dtaudpTrj = _sharedDID->getdTaudp(x);
			const vector<vector<MatrixX>>& d2taudp2Trj = _sharedDID->getd2Taudp2(x);
			shared_ptr<MotorJoint> tempJointPtr;
			Real voltage;
			Real current;
			vector<Math::MatrixX> val(1, MatrixX(x.size(), x.size()));
			val[0].setZero();
			MatrixX dcurrentdp(1, x.size());
			MatrixX dvoltagedp(1, x.size());
			for (int i = 0; i < tauTrj.cols(); i++)
			{
				for (int j = 0, dofIdx = 0; j < _sharedDID->_socAssem->getMateList().size(); j++)
				{
					tempJointPtr = static_pointer_cast<MotorJoint>(_sharedDID->_socAssem->getJointPtrByMateIndex(j));
					for (int k = 0; k < tempJointPtr->getDOF(); k++, dofIdx++)
					{
						current = 1.0 / (tempJointPtr->getMotorConstant() * tempJointPtr->getGearRatio()) * tauTrj(dofIdx, i)
							+ tempJointPtr->getRotorInertia() / tempJointPtr->getMotorConstant() * _sharedDID->_stateTrj[i]->getJointStateByMateIndex(j).getqddot()(k);
						voltage = current * tempJointPtr->getResistance() + tempJointPtr->getBackEMFConstant() * tempJointPtr->getGearRatio() * _sharedDID->_stateTrj[i]->getJointStateByMateIndex(j).getqdot()(k);

						if (current * voltage > 0.0)
						{
							dcurrentdp = 1.0 / (tempJointPtr->getMotorConstant() * tempJointPtr->getGearRatio()) * dtaudpTrj[i].row(dofIdx)
								+ tempJointPtr->getRotorInertia() / tempJointPtr->getMotorConstant() * _sharedDID->_dqdp[i].row(dofIdx);
							dvoltagedp = tempJointPtr->getResistance() * dcurrentdp
								+ tempJointPtr->getBackEMFConstant()*tempJointPtr->getGearRatio() * _sharedDID->_dqdotdp[i].row(dofIdx);
							val[0] += _sharedDID->_dt * ((current * tempJointPtr->getResistance() + voltage) / (tempJointPtr->getMotorConstant() * tempJointPtr->getGearRatio()) * d2taudp2Trj[i][dofIdx]
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
			val.block(0, 0, linearIneq.size(), 1) = linearIneq;
			val.block(linearIneq.size(), 0, nonlinearIneq.size(), 1) = nonlinearIneq;
			return val;
		}

		Math::MatrixX BSplinePointToPointOptimization::inequalityConstraint::Jacobian(const Math::VectorX & x) const
		{
			MatrixX linearIneq = (*_linearIneqConstraint).Jacobian(x);
			MatrixX nonlinearIneq = (*_nonLinearIneqConstraint).Jacobian(x);
			MatrixX val(linearIneq.rows() + nonlinearIneq.rows(), x.size());
			val.block(0, 0, linearIneq.size(), x.size()) = linearIneq;
			val.block(linearIneq.size(), 0, nonlinearIneq.size(), x.size()) = nonlinearIneq;
			return val;
		}

		vector<Math::MatrixX> BSplinePointToPointOptimization::inequalityConstraint::Hessian(const Math::VectorX & x) const
		{
			vector<MatrixX> val = (*_linearIneqConstraint).Hessian(x);
			vector<MatrixX> nonlinearIneq = (*_nonLinearIneqConstraint).Hessian(x);
			val.insert(val.end(), nonlinearIneq.begin(), nonlinearIneq.end());
			return val;
		}

		BSplinePointToPointOptimization::nonLinearInequalityConstraint::nonLinearInequalityConstraint()
		{

		}

		Math::VectorX BSplinePointToPointOptimization::nonLinearInequalityConstraint::func(const Math::VectorX & x) const
		{
			const MatrixX& tauTrj = _sharedDID->getTau(x);
			Math::VectorX val(2 * tauTrj.rows());

			for (int i = 0; i < tauTrj.rows(); i++)
			{
				val(i) = tauTrj.row(i).maxCoeff() - _tauMax(i);
				val(i + tauTrj.rows()) = _tauMin(i) - tauTrj.row(i).minCoeff();
			}

			return val;
		}

		Math::MatrixX BSplinePointToPointOptimization::nonLinearInequalityConstraint::Jacobian(const Math::VectorX & x) const
		{
			const MatrixX& tauTrj = _sharedDID->getTau(x);
			const vector<MatrixX>& dtaudpTrj = _sharedDID->getdTaudp(x);
			Math::MatrixX val(2 * tauTrj.rows(), x.size());
			int index;
			for (int i = 0; i < tauTrj.rows(); i++)
			{
				tauTrj.row(i).maxCoeff(&index);
				val.row(i) = dtaudpTrj[index].row(i);
				tauTrj.row(i).minCoeff(&index);
				val.row(i + tauTrj.rows()) = - dtaudpTrj[index].row(i);
			}

			return val;
		}
		vector<Math::MatrixX> BSplinePointToPointOptimization::nonLinearInequalityConstraint::Hessian(const Math::VectorX & x) const
		{
			const MatrixX& tauTrj = _sharedDID->getTau(x);
			const vector<vector<MatrixX>>& dtau2dp2Trj = _sharedDID->getd2Taudp2(x);
			vector<Math::MatrixX> val(2 * tauTrj.rows(), MatrixX(x.size(), x.size()));
			int index;
			for (int i = 0; i < tauTrj.rows(); i++)
			{
				tauTrj.row(i).maxCoeff(&index);
				val[i] = dtau2dp2Trj[index][i];
				tauTrj.row(i).minCoeff(&index);
				val[i + tauTrj.rows()] = - dtau2dp2Trj[index][i];
			}

			return val;
		}
	}
}

