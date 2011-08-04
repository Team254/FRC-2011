#ifndef ELEVATOR_CONTROL_H
#define ELEVATOR_CONTROL_H

#include "RobotState.h"
#include "CSVReader.h"

/**
 * Controls the elevator of the robot
 */
class ElevatorControl
{
public:
	ElevatorControl(RobotState* robot, CSVReader* csvReader);
	void Update();
	void Reset();
private:
	RobotState* m_robot;
	CSVReader* m_csvReader;
	double m_last_elevator_height;
	double m_elevator_integral;
};

#endif // ELEVATOR_CONTROL_H

