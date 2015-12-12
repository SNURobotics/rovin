#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>

#include <rovin/Reflexxes/Core.h>


//*************************************************************************
// defines

#define CYCLE_TIME_IN_SECONDS                   0.006
#define NUMBER_OF_DOFS                          6
using namespace std;
int main()
{
	// ********************************************************************
	// Variable declarations and definitions
	const double	PI = 3.14159265358979323846;
	const double	DEG2RAD = PI / 180.0;
	const double	RAD2DEG = 180.0 / PI;

	int                         ResultValue = 0;

	ReflexxesAPI                *RML = NULL;

	RMLPositionInputParameters  *IP = NULL;

	RMLPositionOutputParameters *OP = NULL;

	RMLPositionFlags            Flags;
	Flags.SynchronizationBehavior = RMLPositionFlags::PHASE_SYNCHRONIZATION_IF_POSSIBLE;
	// ********************************************************************
	// Creating all relevant objects of the Type II Reflexxes Motion Library

	RML = new ReflexxesAPI(NUMBER_OF_DOFS
						   , CYCLE_TIME_IN_SECONDS);

	IP = new RMLPositionInputParameters(NUMBER_OF_DOFS);

	OP = new RMLPositionOutputParameters(NUMBER_OF_DOFS);

	// ********************************************************************
	// Set-up a timer with a period of one millisecond
	// (not implemented in this example in order to keep it simple)
	// ********************************************************************

	printf("-------------------------------------------------------\n");
	printf("Reflexxes Motion Libraries                             \n");
	printf("Example: 01_RMLPositionSampleApplication               \n\n");
	printf("This example demonstrates the most basic use of the    \n");
	printf("Reflexxes API (class ReflexxesAPI) using the position- \n");
	printf("based Type II Online Trajectory Generation algorithm.  \n\n");
	printf("Copyright (C) 2014 Google, Inc.                      \n");
	printf("-------------------------------------------------------\n");

	//	q(0)
	IP->CurrentPositionVector->VecData[0] = 100.0	*	DEG2RAD;
	IP->CurrentPositionVector->VecData[1] = 0.0		*	DEG2RAD;
	IP->CurrentPositionVector->VecData[2] = 50.0	*	DEG2RAD;
	IP->CurrentPositionVector->VecData[3] = 100.0	*	DEG2RAD;
	IP->CurrentPositionVector->VecData[4] = 0.0		*	DEG2RAD;
	IP->CurrentPositionVector->VecData[5] = 50.0	*	DEG2RAD;

	//	q(t_f)
	IP->TargetPositionVector->VecData[0] = -600.0;
	IP->TargetPositionVector->VecData[1] = -200.0;
	IP->TargetPositionVector->VecData[2] = -350.0;
	IP->TargetPositionVector->VecData[3] = -600.0;
	IP->TargetPositionVector->VecData[4] = -200.0;
	IP->TargetPositionVector->VecData[5] = -350.0;

	//	dq(0), ddq(0), dq(t_f)
	for (int i = 0; i < NUMBER_OF_DOFS; i++)
	{
		IP->CurrentVelocityVector->VecData[i] = 0;
		IP->CurrentAccelerationVector->VecData[i] = 0;
		IP->TargetVelocityVector->VecData[i] = 0;
	}

	//	max dq
	IP->MaxVelocityVector->VecData[0] = 100.0		*	DEG2RAD;
	IP->MaxVelocityVector->VecData[1] = 80.0		*	DEG2RAD;
	IP->MaxVelocityVector->VecData[2] = 140.0		*	DEG2RAD;
	IP->MaxVelocityVector->VecData[3] = 290.0		*	DEG2RAD;
	IP->MaxVelocityVector->VecData[4] = 290.0		*	DEG2RAD;
	IP->MaxVelocityVector->VecData[5] = 440.0		*	DEG2RAD;

	//	max ddq, dddq
	for (int i = 0; i < NUMBER_OF_DOFS; i++)
	{
		IP->MaxAccelerationVector->VecData[i] = IP->MaxVelocityVector->VecData[i] * 5;
		IP->MaxJerkVector->VecData[i] = 1e+10;
		IP->SelectionVector->VecData[i] = true;
	}

	//	outFile open
	ofstream outFile("traj_reflexxes.txt");

	// ********************************************************************
	// Starting the control loop

	while (ResultValue != ReflexxesAPI::RML_FINAL_STATE_REACHED)
	{

		// ****************************************************************
		// Wait for the next timer tick
		// (not implemented in this example in order to keep it simple)
		// ****************************************************************

		for (int i = 0; i < NUMBER_OF_DOFS; i++)
			outFile << IP->CurrentPositionVector->VecData[i] << '\t';
		outFile << '\n';


		// Calling the Reflexxes OTG algorithm
		ResultValue = RML->RMLPosition(*IP, OP, Flags);

		if (ResultValue < 0)
		{
			printf("An error occurred (%d).\n", ResultValue);
			break;
		}



		*IP->CurrentPositionVector = *OP->NewPositionVector;
		*IP->CurrentVelocityVector = *OP->NewVelocityVector;
		*IP->CurrentAccelerationVector = *OP->NewAccelerationVector;
	}

	// ********************************************************************
	// Deleting the objects of the Reflexxes Motion Library end terminating
	// the process

	delete  RML;
	delete  IP;
	delete  OP;

	exit(EXIT_SUCCESS);
}
