#ifndef ROLLER_CONTROL_H
#define ROLLER_CONTROL_H

#include "RobotState.h"
#include "CSVReader.h"

/**
 * Controls the rollers of the robot
 */
class RollerControl
{
public:
	RollerControl(RobotState* robot, CSVReader* csvReader);
	void Update();
	void Reset();
private:
	RobotState* m_robot;
	CSVReader* m_csvReader;
};

#endif // ROLLER_CONTROL_H

