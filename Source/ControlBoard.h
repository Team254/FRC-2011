#ifndef CONTROL_BOARD_H
#define CONTROL_BOARD_H

#include "WPILib.h"

class RobotState;
class CSVReader;

/**
 * Interface for the Control Board
 * Sets desired inputs into the Robot
 * State to be handled by the Control
 * Loops.
 **/
class ControlBoard {
	
public:
	ControlBoard(RobotState* robotState, CSVReader* csvReader);
	~ControlBoard() {}

	/**
	 * Reads values from input and updates data member values.
	 */
	void UpdateValues();
	void CancelSemiAutoActions();
	void ResetToggleSwitches();
	void ResetState();
	
private:
	
	//TODO(ebakan): need to add leds?
	
	// input enums
	enum ControlBoardButton
	{
		kMinibotReleaseButton,
		kMinibotDeployButton,
		kArmUpButton,
		kArmDownButton,
		kLowButton,
		kLowUpButton,
		kMiddleButton,
		kMiddleUpButton,
		kTopButton,
		kTopUpButton,
		kRollInButton,
		kRollOutButton,
		kStow,
		kScoreSwitch,
		kScoreButton,
		kGroundPickup,
		kOperatorScore,
		kSensorCancelButton,
		kSingleSelect,
		kDoubleLeftSelect,
		kDoubleRightSelect,
		kNumControlBoardButtons
	};
	
	enum ControlBoardLEDButton
	{
		kRedLEDButton,
		kBlueLEDButton,
		kWhiteLEDButton,
		kNumControlBoardLEDS
	};
	
	enum JoystickButton {
		kLeftJoystickButton,
		kLeftJoystickSwitch,
		kLeftJoystickTrigger,
		kRightJoystickButton,
		kRightJoystickSwitch,
		kRightJoystickTrigger,
		kNumJoystickButtons
	};
	
	enum JoystickAxis {
		kLeftJoystickYAxis,
		kRightJoystickXAxis,
		kElevatorJoystickAxis,
		kRollerJoystickAxis,
		kNumJoystickAxes
	};
	
	// drivetrain
	double GetStraightDrivePower();
	double GetTurnPower();
	bool WantQuickTurn();
	bool WantHighGear();
	
	// elevator
	double GetElevatorPower();
	bool hasElevatorUserInput();
	void SetElevatorHeight();
	
	// arm
	double GetArmPower();
	
	// rollers
	double GetRollerSpinVal();
	bool WantGrabberOpen();
	bool WantRollIn();
	bool WantRollOut();
	
	// other
	bool ReleaseMinibot();
	bool DeployMinibot();
	bool ControlLoopsOn();
	bool AutoScoreOn();
	void AutoScore();
	
	// helper functions
	bool getJoystickButton(JoystickButton button);
	bool getBoardButton(ControlBoardButton button);
	bool getLED(ControlBoardLEDButton led);
	bool CalculateArmButton(ControlBoardButton button);
	double getJoystickAxisValue(JoystickAxis axis);
	double handleDeadband(double val, double deadband);
	
	// data members
	RobotState* m_robot;
	CSVReader* m_csvReader;
	Joystick m_LeftJoystick;
	Joystick m_RightJoystick;
	Joystick m_ESTOPBoard;
	bool m_initialMinibotRelease;
	bool m_initialMinibotDeploy;
	bool m_initialAutoScoreStatus;
	bool m_autoScoreFlipped;
	
	// preset height vars
	bool m_arm_is_set_up;
	bool m_elevator_set_up;	
	
	// autoscore vars
	enum AutoScoreState {
		lowering_elevator,
		decrementing_elevator_height_first,
		lowering_elevator_second,
		decrementing_elevator_height_second,
		backing_away,
		waiting_to_complete
	};	
	// autoscore
	double m_rollout_time;
	bool m_low_peg;
	
	// arm
	double m_prevArmPower;
	
	// joystick
	double m_prevElevatorJoystick;
	
	// autoscore
	double m_elevator_destination;
	double m_elevator_decrement;
	AutoScoreState m_autoScoreState;
	
	
};

#endif // CONTROL_BOARD_H
