#ifndef ARM_CONTROL_H
#define ARM_CONTROL_H

#include "RobotState.h"
#include "CSVReader.h"

/**
 * Controls the arm of the robot
 */
class ArmControl
{
public:
	ArmControl(RobotState* robot, CSVReader* csvReader);
	void Update();
	void Reset();
private:
	RobotState* m_robot;
	CSVReader* m_csvReader;
	double m_last_arm_angle;
};

#endif // ARM_CONTROL_H

