
//look at WPIlib notifier class

#ifndef CONTROL_LOOPS_H
#define CONTROL_LOOPS_H

#include "WPILib.h"
#include "ControlLoops/DriveControl.h"
#include "ControlLoops/ArmControl.h"
#include "ControlLoops/RollerControl.h"
#include "ControlLoops/ElevatorControl.h"
#include "ControlLoops/MinibotControl.h"

class RobotState;
class CSVReader;

/**
 * Contains references to control loops for the arm, drive, roller,
 * elevator, and minibot.
 *
 * This coordinates the updates of all the robot's separate control
 * loops simultaneously.
 */
class ControlLoops {
public:
	ControlLoops(RobotState* robot, CSVReader* csvReader);
	~ControlLoops();

	/**
	 * Resets (if needed) and then updates each of the referenced control loops.
	 */
	void UpdateControlLoops();

	/**
	 * Marks the control loops for reset on next update.
	 */
	void Reset();
private:
	bool shouldReset;
	// data members
	Notifier* m_notifier;
	RobotState* m_robot;
	CSVReader* m_csvReader;
	DriveControl* m_driveControl;
	RollerControl* m_rollerControl;
	ArmControl* m_armControl;
	ElevatorControl* m_elevatorControl;
	MinibotControl* m_minibotControl;
};
#endif // CONTROL_LOOPS_H
