#pragma once

#include "RevoluteJoint.h"

namespace rovin
{
	namespace Model
	{
		class MotorJoint : public RevoluteJoint
		{
		public:
			MotorJoint(const std::string& name) : RevoluteJoint(name) {}

			void setResistance(const Math::Real value) { _resistance = value; }
			void setInductance(const Math::Real value) { _inductance = value; }
			void setRotorInertia(const Math::Real value) { _rotorInertia = value; }
			void setGearRatio(const Math::Real value) { _gearRatio = value; }
			void setMotorConstant(const Math::Real value) { _motorConstant = value; }
			void setBackEMFConstant(const Math::Real value) { _backEMFConstant = value; }

			Math::Real getResistance() const { return _resistance; }
			Math::Real getInductance() const { return _inductance; }
			Math::Real getRotorInertia() const { return _rotorInertia; }
			Math::Real getGearRatio() const { return _gearRatio; }
			Math::Real getMotorConstant() const { return _motorConstant; }
			Math::Real getBackEMFConstant() const { return _backEMFConstant; }

		private:
			Math::Real _resistance;
			Math::Real _inductance;
			Math::Real _rotorInertia;
			Math::Real _gearRatio;
			Math::Real _motorConstant;
			Math::Real _backEMFConstant;
		};
	}
}