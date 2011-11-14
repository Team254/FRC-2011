#include "WPILib.h"
#include "semLib.h"

#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

class CSVReader;

/**
 * Abstraction layer over a robot.
 * Stores inputted data from the ControlBoard
 * and provides methods for the Control Loops
 * for getting sensor input and setting
 * output values.
 **/
class RobotState {
public:
	/**
	 * Constructor that initializes inputs to the robot.
	 *
	 * @param csvReader a CSV reader providing constants to the robot.
	 */
	RobotState(CSVReader* csvReader);
	~RobotState();
		
	// semaphore
	SEM_ID m_lock;
	
	//Sensor getters
	double GetLeftDistance() const;
	double GetRightDistance() const;
	double GetGyroValue() const;
	double GetArmAngle() const;
	double GetElevatorHeightValue() const;
	bool GetTubeLimitSwitch() const;
	bool GetCarriageLimitSwitch() const;
	bool GetSecondStageLimitSwitch() const;
	
	//Output setters
	void SetLeftMotor(double pwm);
	void SetLeftMotor_Linearized(double pwm);
	void SetRightMotor(double pwm);
	void SetRightMotor_Linearized(double pwm);
	void SetElevatorMotor(double pwm);
	void SetElevatorMotor_Linearized(double pwm);
	void SetArmMotor(double pwm);
	void SetArmMotor_Linearized(double pwm);
	void SetTopRollerMotor(double pwm);
	void SetTopRollerMotor_Linearized(double pwm);
	void SetBottomRollerMotor(double pwm);
	void SetBottomRollerMotor_Linearized(double pwm);
	void SetMinibotRelease(bool on);
	void SetMinibotDeploy(bool on);
	void SetHighGear(bool high);
	void SetGrabberOpen(bool open);
	
	// Auto mode helpers.
	void waitForArm();
	void waitForArmWithTimeout(double time);
	void waitForElevator();
	void armAngleDegrees(double angle);
	void elevatorHeight(double height);
	void closeClaw();
	void grabTube();
	void openClaw();
	void ejectTube();
	void ignoreTube();
	bool hasTube();
	void waitForTube();
	void shiftHighGear();
	void shiftLowGear();
	
	//Resetters
	void ResetState();
	void ResetEncoders();
	void ResetGyro();
	
	//Others
	void EnableLeftEncoder();
	void DisableLeftEncoder();
	void ResetLeftEncoder();
	
	void EnableRightEncoder();
	void DisableRightEncoder();
	void ResetRightEncoder();
	
	void EnableElevatorEncoder();
	void DisableElevatorEncoder();
	void ResetElevatorEncoder();
	
	void EnableCompressor();
	void DisableCompressor();
	
	double GetTime();
	//threadsafe version
	double threadsafeTime();
	void ResetTimer();

	// PID loop (ensures steady motor output)
	double victor_linearize(double goal_speed);

	// synchronization
	void Lock();
	void Unlock();
	
	
	// INPUTS FROM CONTROL BOARD
	
	// drivetrain
	double straightDrivePower;
	double turnPower;
	bool isQuickTurn;
	bool isHighGear;

	bool isControlLoopDriving;
	//bool isLeftRight;
	
	double straightDistanceGoal;
	double straightDistanceGoalVelocity;
	double straightDistanceMaxVelocity;
	double straightDistanceMaxAcceleration;
	
	bool resetDriveControl;
	bool ignoreTurnControlLoop;
	double turnOffset;
	
	double turnAngleGoal;
	double turnAngleGoalVelocity;
	bool driveStopped();
	void enableSteering(bool isEnabled);
	bool driveStoppedFine();
	bool driveStoppedWithinDegrees(double deg);
	void waitForDrive();
	void waitForDriveFine();
	void waitForDriveWithinDegrees(double deg);
	void waitForDriveFeetLeft(double feet);
	void driveForwardsFeet(double feet);
	void resetDriveGoal();
	void driveForwardsFeetAtVelocity(double feet, double velocity);
	void turnAngleDegrees(double angle);
	
	// elevator
	double elevatorPower;
	double elevatorGoal; //set to the desired height if a preset button is pressed, or set to -1 if manual control is desired
	double elevatorVelocity;
	bool isZeroingElevator;
	bool elevatorStopped();
	bool elevatorEnabled;
	bool isGrounding;
	bool isSetpointing;
	
	// arm
	// Power to apply to the arm if it is disabled.
	double armPower;
	// Goal for the control loops in radians.
	double armGoal;
	double armVelocity;
	bool armStopped();
		
	// rollers
	double rollerSpinVal;
	bool grabberOpen;
	
	// roller states
	bool isRollingIn;
	bool isRollingOut;
	
	//other
	bool minibotRelease;
	bool minibotDeploy;
	bool controlLoopsOn;
	bool isOperatorControl;
	bool isAutonomous;
	
	
	//assumed position and angle
	void updateAssumption();
	double assumed_xpos;
	double assumed_ypos;
	double assumed_theta;
	double assumedTurnOffset;
	//thread-safe getters
	double GetAssumedXPos();
	double GetAssumedYPos();
	double GetAssumedTheta();
	

	// Timestep size!
	const static double dt = 0.01;
	
	enum AutonomousState
		{
			kNoTube = 0,
			kOneTube,
			kTwoTubeLeft,
			kTwoTubeRight,
			kNumAutonomousModes
		};
		
		AutonomousState currAutonomous;
		
		char* autonomousDescription;

	
	bool	isDisabled;
	//we're only using one lcd pointer shared across all
	//classes for threadsafeness. In theory this shouldn't
	//matter, but better safe than sorry
	DriverStationLCD* m_lcd;
	
private:
	// data members
	
	// drivetrain
	Victor* m_leftMotor1;
	Victor* m_leftMotor2;
	Victor* m_rightMotor1;
	Victor* m_rightMotor2;
	Encoder* m_leftEncoder;
	Encoder* m_rightEncoder;
	Solenoid* m_gearShiftSolenoid;
	
	// elevator
	Victor* m_elevatorMotor1;
	Victor* m_elevatorMotor2;
	Encoder* m_elevatorEncoder;
	DigitalInput* m_tubeSwitch;
	DigitalInput* m_secondStageSwitch;
	DigitalInput* m_carriageSwitch;
	
	// arm
	Victor* m_armMotor;
	AnalogChannel* m_armPot;
	DoubleSolenoid* m_grabberSolenoid;
	
	// rollers
	Victor* m_topRollerMotor;
	Victor* m_bottomRollerMotor;
	
	// other
	Timer* m_timer;
	Compressor* m_compressor;
	Solenoid* m_minibotReleaseSolenoid;
	Solenoid* m_minibotDeploySolenoid;
	Gyro* m_gyro;
	
	double m_ArmHighVoltage;
	double m_ArmLowVoltage;
	
	CSVReader* m_csvReader;
	
	double prev_left;
	double prev_right;
	double prev_angle;
};

#endif // ROBOT_STATE_H
