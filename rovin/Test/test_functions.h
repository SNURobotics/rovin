#pragma once
#include <iostream>

#include <rovin/Math/Common.h>
#include <rovin/Math/Constant.h>
#include <rovin/Model/Assembly.h>
#include <rovin/Model/RevoluteJoint.h>
#include <rovin/Model/MotorJoint.h>
#include <rovin/Dynamics/Kinematics.h>
#include <rovin/Dynamics/Dynamics.h>
#include <rovin/Math/Optimization.h>
#include <rovin/utils/Diagnostic.h>


using namespace std;
using namespace rovin;
using namespace rovin::Math;
using namespace rovin::Model;

VectorX calcEffort(const MatrixX& jointTorqueTrj)
{
	VectorX effortTrj(jointTorqueTrj.cols());
	for (int i = 0; i < jointTorqueTrj.cols(); i++)
		effortTrj(i) = jointTorqueTrj.col(i).squaredNorm();
	return effortTrj;
};

VectorX calcEnergy(const socAssembly& socAssem, const MatrixX& jointVelTrj, const MatrixX& jointAccTrj, const MatrixX& jointTorqueTrj)
{
	shared_ptr<MotorJoint> tempJointPtr;
	Real voltage;
	Real current;
	VectorX energyConsumptionTrj(jointTorqueTrj.cols());
	energyConsumptionTrj.setZero();
	for (int i = 0; i < jointTorqueTrj.cols(); i++)
	{
		for (unsigned int j = 0; j < socAssem.getMateList().size(); j++)
		{
			tempJointPtr = static_pointer_cast<MotorJoint>(socAssem.getJointPtrByMateIndex(j));
			current = 1.0 / (tempJointPtr->getMotorConstant() * tempJointPtr->getGearRatio()) * jointTorqueTrj(j, i)
				+ tempJointPtr->getRotorInertia() * tempJointPtr->getGearRatio() / tempJointPtr->getMotorConstant() * jointAccTrj(j, i);
			voltage = current * tempJointPtr->getResistance() + tempJointPtr->getBackEMFConstant() * tempJointPtr->getGearRatio() * jointVelTrj(j, i);
			energyConsumptionTrj(i) += max(current * voltage, 0.0);
		}
	}
	return energyConsumptionTrj;
}

MatrixX calcTorque(const socAssembly& socAssem, const MatrixX& pos, const MatrixX& vel, const MatrixX& acc)
{
	auto state = socAssem.makeState();
	MatrixX torque; torque.resizeLike(pos);
	for (int i = 0; i < pos.cols(); i++)
	{
		state->setJointq(State::STATEJOINT, pos.col(i));
		state->setJointqdot(State::STATEJOINT, vel.col(i));
		state->setJointqddot(State::STATEJOINT, acc.col(i));
		Dynamics::solveInverseDynamics(socAssem, *state);

		torque.col(i) = state->getJointTorque(State::STATEJOINT);
	}
	return torque;
}