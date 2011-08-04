#ifndef MINIBOT_CONTROL_H
#define MINIBOT_CONTROL_H

#include "RobotState.h"
#include "CSVReader.h"

/**
  * Governs when the minibot is deployed
  * and released
  */

class MinibotControl
{
public:
	MinibotControl(RobotState* robot, CSVReader* csvReader);
	void Update();
	void Reset();
private:
	RobotState* m_robot;
	CSVReader* m_csvReader;
};

#endif // MINIBOT_CONTROL_H

