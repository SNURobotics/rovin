#pragma once

#include "Constant.h"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>

namespace rovin
{
	namespace Math
	{
		static const unsigned int SizeOfLookUpTable = 4000;
		static const Real StepSizeOfLookUpTable = PI_HALF / (SizeOfLookUpTable - 1);
		static const Real InvStepSizeOfLookUpTable = 1 / StepSizeOfLookUpTable;
		static Real sineLookupTalbe[SizeOfLookUpTable] = {
#include "sineLookUpTable"
		};
		static Real sineLookupTalbe_Gradient[SizeOfLookUpTable] = {
#include "sineLookUpTable_Gradient"
		};
		static Real sineLookupTalbe_Delta[SizeOfLookUpTable] = {
#include "sineLookUpTable_Delta"
		};

		static void makeSineLookUpTable()
		{
			std::ofstream output_value, output_gradient, output_delta;
			output_value.open("..\\Math\\sineLookUpTable");
			output_gradient.open("..\\Math\\sineLookUpTable_Gradient");
			output_delta.open("..\\Math\\sineLookUpTable_Delta");
			for (unsigned int i = 0; i < SizeOfLookUpTable; i++)
			{
				if (i != 0 && i % 50 == 0) output_value << "\n";
				if (i != 0) output_value << ", ";
				output_value << std::setprecision(15) << sin(PI_HALF / (SizeOfLookUpTable - 1) * (Real)i);

				if (i != 0 && i % 50 == 0) output_gradient << "\n";
				if (i != 0) output_gradient << ", ";
				output_gradient << std::setprecision(15) << (sin(PI_HALF / (SizeOfLookUpTable - 1) * (Real)(i + 1)) - sin(PI_HALF / (SizeOfLookUpTable - 1) * (Real)i))/StepSizeOfLookUpTable;

				if (i != 0 && i % 50 == 0) output_delta << "\n";
				if (i != 0) output_delta << ", ";
				output_delta << std::setprecision(15) << (sin(PI_HALF / (SizeOfLookUpTable - 1) * (Real)(i + 1)) - sin(PI_HALF / (SizeOfLookUpTable - 1) * (Real)i)) * i;
			}
		}

		static Real fsin(Real theta)
		{
			theta -= (int)(theta*Inv_PI_DOUBLE)*PI_DOUBLE;
			if (theta < 0) theta += PI_DOUBLE;

			unsigned int x = (unsigned int)(theta*InvStepSizeOfLookUpTable);

			if (x < SizeOfLookUpTable)
			{
				return sineLookupTalbe_Gradient[x] * theta - sineLookupTalbe_Delta[x] + sineLookupTalbe[x];
			}
			else if (x < SizeOfLookUpTable * 2)
			{
				theta = PI - theta;
				x = SizeOfLookUpTable * 2 - x - 1;
				return sineLookupTalbe_Gradient[x] * theta - sineLookupTalbe_Delta[x] + sineLookupTalbe[x];
			}
			else if (x < SizeOfLookUpTable * 3)
			{
				theta -= PI;
				x -= SizeOfLookUpTable * 2;
				return sineLookupTalbe_Delta[x] - sineLookupTalbe[x] - sineLookupTalbe_Gradient[x] * theta;
			}
			theta = PI_DOUBLE - theta;
			x = SizeOfLookUpTable * 4 - x - 1;
			return sineLookupTalbe_Delta[x] - sineLookupTalbe[x] - sineLookupTalbe_Gradient[x] * theta;
		}

		static Real fcos(Real theta)
		{
			return fsin(theta + PI_HALF);
		}
	}
}