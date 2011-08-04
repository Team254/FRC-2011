#ifndef TEST_CONTROL_H
#define TEST_CONTROL_H

#include "RobotState.h"
#include "CSVReader.h"
#include "DiscreteAccelFilter.h"
#include "ContinuousAccelFilter.h"

/**
 * Controls the drive of the robot
 * Just a test, unused on the robot
 */
#define NUM_STATES 4
#define NUM_INPUTS 2
#define NUM_OUTPUTS 2

class TestControl
{
public:
	void mult(double dest[DIM], double A[DIM][DIM], double x[DIM]);
private:
	double A[NUM_STATES][NUM_STATES];
	double K[NUM_STATES][NUM_STATES];
	double L[NUM_STATES][NUM_OUTPUTS];
	double C[NUM_OUTPUTS][NUM_STATES];
	double D[NUM_OUTPUTS][NUM_STATES];
	double x[NUM_STATES];
	double r[NUM_STATES];
	double y[NUM_OUTPUTS];
	double u[NUM_OUTPUTS];
}
