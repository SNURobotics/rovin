#pragma once
#include "./include/ReflexxesAPI.h"
#include "./include/RMLPositionFlags.h"
#include "./include/RMLPositionInputParameters.h"
#include "./include/RMLPositionOutputParameters.h"

#include <vector>

#include <Eigen/Dense>
#include <rovin\Math\Constant.h>

using namespace rovin::Math;

namespace Reflexxes
{
	class ReflexxesWrapper
	{
	public:
		ReflexxesWrapper(unsigned int DOF, double timeStep)
			:_DOF(DOF), _RML(DOF, timeStep), _IP(DOF), _OP(DOF)
		{
			_Flag.SynchronizationBehavior = RMLPositionFlags::PHASE_SYNCHRONIZATION_IF_POSSIBLE;
			
			_q0.setZero(DOF);
			_dq0.setZero(DOF);
			_ddq0.setZero(DOF);

			_qf.setZero(DOF);
			_dqf.setZero(DOF);

			_dqLim = VectorX::Ones(DOF) * RealMax;
			_ddqLim = VectorX::Ones(DOF) * RealMax;
			_dddqLim = VectorX::Ones(DOF) * RealMax;

			_qResult.resize(_DOF, 1);
			_dqResult.resize(_DOF, 1);
			_ddqResult.resize(_DOF, 1);
		}

		const MatrixX&	solve()
		{
			for (int i = 0; i < _DOF; i++)
			{
				_IP.CurrentPositionVector->VecData[i] = _q0[i];
				_IP.CurrentVelocityVector->VecData[i] = _dq0[i];
				_IP.CurrentAccelerationVector->VecData[i] = _ddq0[i];

				_IP.TargetPositionVector->VecData[i] = _qf[i];
				_IP.TargetVelocityVector->VecData[i] = _dqf[i];

				_IP.MaxVelocityVector->VecData[i] = _dqLim[i];
				_IP.MaxAccelerationVector->VecData[i] = _ddqLim[i];
				_IP.MaxJerkVector->VecData[i] = _dddqLim[i];

				_IP.SelectionVector->VecData[i] = true;
			}

			int	ResultValue = 0;
			int count = 0;

			//	put initial state.
			for (int i = 0; i < _DOF; i++)
			{
				_qResult(i, count) = _IP.CurrentPositionVector->VecData[i];
				_dqResult(i, count) = _IP.CurrentVelocityVector->VecData[i];
				_ddqResult(i, count) = _IP.CurrentAccelerationVector->VecData[i];
			}
			count++;
			while (ResultValue != ReflexxesAPI::RML_FINAL_STATE_REACHED)
			{
				ResultValue = _RML.RMLPosition(_IP, &_OP, _Flag);
				if (ResultValue < 0)
				{
					printf("An error occurred (%d).\n", ResultValue);
					break;
				}

				_IP.CurrentPositionVector = _OP.NewPositionVector;
				_IP.CurrentVelocityVector = _OP.NewVelocityVector;
				_IP.CurrentAccelerationVector = _OP.NewAccelerationVector;

				_qResult.conservativeResize(Eigen::NoChange, count + 1);
				_dqResult.conservativeResize(Eigen::NoChange, count + 1);
				_ddqResult.conservativeResize(Eigen::NoChange, count + 1);

				for (int i = 0; i < _DOF; i++)
				{
					_qResult(i, count) = _IP.CurrentPositionVector->VecData[i];
					_dqResult(i, count) = _IP.CurrentVelocityVector->VecData[i];
					_ddqResult(i, count) = _IP.CurrentAccelerationVector->VecData[i];
				}

				count++;
			}
			

			return _qResult;
		}

		std::vector<MatrixX>	solve2()
		{
			for (int i = 0; i < _DOF; i++)
			{
				_IP.CurrentPositionVector->VecData[i] = _q0[i];
				_IP.CurrentVelocityVector->VecData[i] = _dq0[i];
				_IP.CurrentAccelerationVector->VecData[i] = _ddq0[i];

				_IP.TargetPositionVector->VecData[i] = _qf[i];
				_IP.TargetVelocityVector->VecData[i] = _dqf[i];

				_IP.MaxVelocityVector->VecData[i] = _dqLim[i];
				_IP.MaxAccelerationVector->VecData[i] = _ddqLim[i];
				_IP.MaxJerkVector->VecData[i] = _dddqLim[i];

				_IP.SelectionVector->VecData[i] = true;
			}

			int	ResultValue = 0;
			int count = 0;

			//	put initial state.
			for (int i = 0; i < _DOF; i++)
			{
				_qResult(i, count) = _IP.CurrentPositionVector->VecData[i];
				_dqResult(i, count) = _IP.CurrentVelocityVector->VecData[i];
				_ddqResult(i, count) = _IP.CurrentAccelerationVector->VecData[i];
			}
			count++;
			while (ResultValue != ReflexxesAPI::RML_FINAL_STATE_REACHED)
			{
				ResultValue = _RML.RMLPosition(_IP, &_OP, _Flag);
				if (ResultValue < 0)
				{
					printf("An error occurred (%d).\n", ResultValue);
					break;
				}

				_IP.CurrentPositionVector = _OP.NewPositionVector;
				_IP.CurrentVelocityVector = _OP.NewVelocityVector;
				_IP.CurrentAccelerationVector = _OP.NewAccelerationVector;

				_qResult.conservativeResize(Eigen::NoChange, count + 1);
				_dqResult.conservativeResize(Eigen::NoChange, count + 1);
				_ddqResult.conservativeResize(Eigen::NoChange, count + 1);

				for (int i = 0; i < _DOF; i++)
				{
					_qResult(i, count) = _IP.CurrentPositionVector->VecData[i];
					_dqResult(i, count) = _IP.CurrentVelocityVector->VecData[i];
					_ddqResult(i, count) = _IP.CurrentAccelerationVector->VecData[i];
				}

				count++;
			}

			vector<MatrixX> result(3);
			result[0] = _qResult;
			result[1] = _dqResult;
			result[2] = _ddqResult;


			return result;
		}
		
		VectorX							_q0, _dq0, _ddq0;
		VectorX							_qf, _dqf;
		VectorX							_dqLim, _ddqLim, _dddqLim;
		
	protected:
		int								_DOF;
		ReflexxesAPI					_RML;
		RMLPositionInputParameters		_IP;
		RMLPositionOutputParameters		_OP;
		RMLPositionFlags				_Flag;

		MatrixX							_qResult;
		MatrixX							_dqResult;
		MatrixX							_ddqResult;
	};

}