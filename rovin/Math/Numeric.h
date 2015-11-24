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
		static const unsigned int SizeOfLookUpTable = 100;
		static const Real StepSizeOfLookUpTable = PI_HALF / (SizeOfLookUpTable - 1);
		static const Real InvStepSizeOfLookUpTable = 1 / StepSizeOfLookUpTable;
		static Real sineLookupTalbe[SizeOfLookUpTable] = {
#include "sineLookUpTable_100"
		};
		static Real sineLookupTalbe_Gradient[SizeOfLookUpTable] = {
#include "sineLookUpTable_Gradient_100"
		};
//		static Real sineLookupTalbe_Delta[SizeOfLookUpTable] = {
//#include "sineLookUpTable_Delta"
//		};
#ifdef LOOKUPTABLE
		static void makeSineLookUpTable()
		{
			std::ofstream output_value, output_gradient;//, output_delta;
			output_value.open("..\\Math\\sineLookUpTable_100");
			output_gradient.open("..\\Math\\sineLookUpTable_Gradient_100");
			//output_delta.open("..\\Math\\sineLookUpTable_Delta");
			for (unsigned int i = 0; i < SizeOfLookUpTable; i++)
			{
				if (i != 0 && i % 50 == 0) output_value << "\n";
				if (i != 0) output_value << ", ";
				output_value << std::setprecision(15) << sin(PI_HALF / (SizeOfLookUpTable - 1) * (Real)i) - ((sin(PI_HALF / (SizeOfLookUpTable - 1) * (Real)(i + 1)) - sin(PI_HALF / (SizeOfLookUpTable - 1) * (Real)i)) *i);

				if (i != 0 && i % 50 == 0) output_gradient << "\n";
				if (i != 0) output_gradient << ", ";
				output_gradient << std::setprecision(15) << (sin(PI_HALF / (SizeOfLookUpTable - 1) * (Real)(i + 1)) - sin(PI_HALF / (SizeOfLookUpTable - 1) * (Real)i))/StepSizeOfLookUpTable;

				//if (i != 0 && i % 50 == 0) output_delta << "\n";
				//if (i != 0) output_delta << ", ";
				//output_delta << std::setprecision(15) << (sin(PI_HALF / (SizeOfLookUpTable - 1) * (Real)(i + 1)) - sin(PI_HALF / (SizeOfLookUpTable - 1) * (Real)i)) * i;
			}
		}
#endif

		static void fsincosTable(Real theta, Real& sine, Real& cosine)
		{
			theta -= (int)(theta*Inv_PI_DOUBLE)*PI_DOUBLE;
			if (theta < 0) theta += PI_DOUBLE;

			unsigned int x = (unsigned int)(theta*InvStepSizeOfLookUpTable), tmpx;

			if (x < SizeOfLookUpTable)
			{
				tmpx = SizeOfLookUpTable - x - 1;
				sine = sineLookupTalbe_Gradient[x] * theta + sineLookupTalbe[x];
				//cosine = sqrt(1 - sine*sine);
				cosine = sineLookupTalbe_Gradient[tmpx] * (PI_HALF - theta) + sineLookupTalbe[tmpx];
			}
			else if (x < SizeOfLookUpTable * 2)
			{
				tmpx = x - SizeOfLookUpTable;
				x = SizeOfLookUpTable * 2 - x - 1;
				sine = sineLookupTalbe_Gradient[x] * (PI - theta) + sineLookupTalbe[x];
				if (sine < -1.0) sine = -1.0;
				else if (sine > 1.0) sine = 1.0;
				//cosine = -sqrt(1 - sine*sine);
				cosine = -sineLookupTalbe_Gradient[tmpx] * (theta - PI_HALF) - sineLookupTalbe[tmpx];
				if (cosine < -1.0) sine = -1.0;
				else if (cosine > 1.0) sine = 1.0;
			}
			else if (x < SizeOfLookUpTable * 3)
			{
				tmpx = SizeOfLookUpTable * 3 - x - 1;
				x -= SizeOfLookUpTable * 2;
				sine = -sineLookupTalbe_Gradient[x] * (theta - PI) - sineLookupTalbe[x];
				//cosine = -sqrt(1 - sine*sine);
				cosine = -sineLookupTalbe_Gradient[tmpx] * (PI_HALF + PI - theta) - sineLookupTalbe[tmpx];
			}
			else
			{
				tmpx = x - SizeOfLookUpTable * 3;
				x = SizeOfLookUpTable * 4 - x - 1;
				sine = -sineLookupTalbe_Gradient[x] * (PI_DOUBLE - theta) - sineLookupTalbe[x];
				//cosine = sqrt(1 - sine*sine);
				cosine = sineLookupTalbe_Gradient[tmpx] * (theta - PI_HALF - PI) + sineLookupTalbe[tmpx];
			}
		}

		static void fsincos(Real theta, Real& sine, Real& cosine)
		{
			theta -= (int)(theta*Inv_PI_DOUBLE)*PI_DOUBLE;
			if (theta < 0) theta += PI_DOUBLE;

			sine = sin(theta);
			if (theta < PI_HALF)
			{
				cosine = sqrt(1 - sine*sine);
				return;
			}
			else if (theta < PI + PI_HALF)
			{
				cosine = -sqrt(1 - sine*sine);
				return;
			}
			cosine = sqrt(1 - sine*sine);
		}
	}
}