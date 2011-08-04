#include <iostream>
#include "WPILib.h"
#include "ControlBoard.h"
#include "ControlLoops.h"
#include "CSVReader.h"
#include "RobotState.h"
#include "Task.h"
#include "CommonIncludes.hpp"
#include <fstream>

/**
  * The main robot class
  * SimpleRobot was chosen because, as its name implies, it is simple.
  * All our control loops run at 100Hz using a Notifier and Autonomous
  * mode is handled with a separate thread, so other, more complex
  * WPIlib robots were deemed unnecessary.
  *
  * Main Components:
  * RobotState represents the current state of the robot - used to share
  *     data between threads and control loops
  *
  * ControlBoard handles interactions with the control board and sets
  *     values in the RobotState
  *     Only updated when a new packet is recieved from the control board
  *
  * ControlLoops is a wrapper for all the control loops of each subsystem,
  *     allows for easy updating of all loops with one call
  *     Runs on a Notifier which activates at a rate of 100Hz
  *
  * CSVReader allows frequently-tweaked values to be stored in a CSV file
  *     which can be easily reloaded (in this case, every time the robot
  *     is disabled and re-enabled) without having to redeploy code
  *
  */
class MainRobot : public SimpleRobot {

public:
	/**
	  * Constructor, initializes all elements of the robot
	  */
	MainRobot();

	/**
	  * Destructor, frees all allocated space to prevent memory leaks
	  */
	~MainRobot();

	/**
	  * Executes when Autonomous mode is running
	  */
	void Autonomous();

	/**
	  * Executes while disabled
	  */
	void Disabled();

	/**
	  * Starts autonomous thread
	  * Called in Autonomous()
	  */
	void RunAutoThread();

	/**
	  * Executes when in Teleop mode
	  */
	void OperatorControl();

private:
	/**
	  * Performs a one-tube autonomous run, reading values from the CSV
	  * Is able to compensate for left- vs right-sided autonomous
	  * by flipping the signs of all the turns
	  *
	  * Designed to be run in its own thread to avoid verbose finite
	  * state machiens
	  */
	void OneTubeAutonomous();

	/**
	  * Performs a two-tube autonomous run, reading values from the CSV
	  * Is able to compensate for left- vs right-sided autonomous
	  * by flipping the signs of all the turns
	  *
	  * Designed to be run in its own thread to avoid verbose finite
	  * state machiens
	  */
	void TwoTubeAutonomous();

	/**
	  * A quick script to print and log various robot state information
	  * along with an informative message
	  * @param msg The message to be included in the output and the log
	  * entry
	  */
	void PrintLogPos(const char* msg);

	/**
	  * Logs various information about the robot state
	  */
	void LogRobotState();

	//one vs two tube autonomous
	uint8_t m_numTubes;

	//left vs right sided autonomous
	bool m_isScoringLeft;

	//used for checking if the control board has been updated
	uint32_t m_prevPacketNumber;

	//controls all interactions with the physical control board
	ControlBoard* m_controlBoard;

	//allows all control loops to be updated from a single place
	ControlLoops* m_controlLoops;

	//allows for easy reloading of values via a CSV file
	CSVReader* m_csvReader;

	//robot state information, used as a shared data space between
	//threads and control loops
	RobotState* m_robot;

	//used for running the Autonomous mode in its own thread
	Task * m_task;

	//local instance of the Driver Station LCD
	DriverStationLCD* m_lcd;

	//the logfile for teleop/all robot activity
	ofstream m_log;

	//the logfile for autonomous mode
	ofstream m_autolog;
};

/**
  * Starts the autonomous mode of the robot
  * @param bot The robot whose autonomous mode should be run
  * @return the exit code: 0 if successful, 1 if failed
  */
int StartTask(MainRobot *bot){
	bot->RunAutoThread();
	return 0;
}

MainRobot::MainRobot()
{
	printf("Constructing the robot\n");

	//init data members
	m_prevPacketNumber = 0;
	m_csvReader = new CSVReader("RobotConfig.csv");
	m_robot= new RobotState(m_csvReader);
	m_controlBoard = new ControlBoard(m_robot, m_csvReader);
	m_controlLoops = new ControlLoops(m_robot, m_csvReader);

	//set up the autonomous Task
	m_task = new Task("AutoMode",(FUNCPTR)StartTask);

	//get a local instance of the Driver Station LCD
	m_lcd = DriverStationLCD::GetInstance();

	//initialize the log files
	//NOTE: To find the most recent entries in a logfile,
	//look for the last occurrence of "Robot Booted" in the file
	m_log.open("log.log",fstream::app);
	m_log << "Robot Booted" << endl << endl;
	m_autolog.open("auto.log",fstream::app);
	m_autolog << "Robot Booted" << endl << endl;

	//make sure to activate the watchdog lest it bite us
	GetWatchdog().SetExpiration(100);
}
MainRobot::~MainRobot()
{
	m_log.close();
	m_autolog.close();
	delete m_controlLoops;
	delete m_controlBoard;
	delete m_robot;
	delete m_csvReader;
	delete m_task;
}

void MainRobot::Disabled()
{
	//when disabled, make sure the RobotState reflects this,
	//but the ControlBoard is still updated to allow for autonomous
	//mode selection
	m_robot->isDisabled = true;
	while(IsDisabled()){
		m_controlBoard->UpdateValues();
	}
}

void MainRobot::Autonomous()
{
	//keep track of starting time to tell when the 15 seconds is up
	//and be sure that logfiles and the RobotState reflect the current
	//status of the robot
	double startTime = m_robot->threadsafeTime();
	GetWatchdog().SetEnabled(false);
	m_csvReader->ReloadValues();
	bool wasDisabled = true;
	bool taskRunning = false;
	m_robot->isOperatorControl=false;
	m_robot->isAutonomous=true;
	m_numTubes=(int)m_csvReader->GetValueWithDefault("NUM_AUTO_TUBES",1);
	m_isScoringLeft=(bool)m_csvReader->GetValue("IS_SCORING_LEFT");
	m_log << "Entering Autonomous Mode" << endl;
	m_log.flush();

	while (IsAutonomous()) {
		m_robot->isDisabled = IsDisabled();
		if (!IsDisabled()) {
			//Autonomous mode is enabled
			if (wasDisabled) {
				m_log << "Enabling Autonomous at time " << m_robot->threadsafeTime() << endl;
				m_log.flush();
				printf("Starting Autonomous Thread\n");
				m_robot->Lock();
				RobotState::AutonomousState currAutonomous = m_robot->currAutonomous;
				m_robot->ResetState();
				m_robot->isOperatorControl=false;
				m_robot->isAutonomous=true;
				m_controlLoops->Reset();
				m_csvReader->ReloadValues();
				m_robot->currAutonomous=currAutonomous;
				//reset the start time
				startTime = m_robot->GetTime();
				m_robot->Unlock();
				//begin the thread
				m_task->Start((int)this);
				wasDisabled = false;
				taskRunning = true;
			}
			//if the timer has run out, kill the thread
			else if ((m_robot->threadsafeTime() - startTime) > 15.0) {
				if (taskRunning) {
					printf("Killing Autonomous Thread\n");
					m_log << "Disabling Autonomous at time " << m_robot->threadsafeTime() << endl << endl;
					m_log.flush();
					m_robot->Lock();
					m_task->Stop();
					m_robot->Unlock();
					m_robot->ResetState();
					taskRunning = false;
				}
				//ensure that all the motors are set to zero
				m_robot->Lock();
				m_robot->SetLeftMotor(0.0);
				m_robot->SetRightMotor(0.0);
				m_robot->SetElevatorMotor(0.0);
				m_robot->SetArmMotor(0.0);
				m_robot->SetTopRollerMotor(0.0);
				m_robot->SetBottomRollerMotor(0.0);
				m_robot->controlLoopsOn = false;
				m_robot->Unlock();
			}
			LogRobotState();
		}
		//the robot is still disabled
		else { 
			//only update the control board when disabled
			m_controlBoard->UpdateValues();
			//if killing the thread, update the logs
			if (!wasDisabled) {
				printf("Killing Autonomous Thread\n");
				m_log << "Disabling Autonomous at time " << m_robot->threadsafeTime() << endl << endl;
				m_log.flush();
				// TODO(aschuh): Lock the csvreader first.
				if (taskRunning) {
					m_robot->Lock();
					m_task->Stop();
					m_robot->Unlock();
					taskRunning = false;
				}
			}
			wasDisabled = true;
		}
		//don't burn up the processor
		Wait(0.01);
	//make sure to update any changes in the LCD
    	m_lcd->UpdateLCD();
	}
	//thread needs to be killed
	//TODO(ebakan): Check if this code is still necessary, when such
	//similar code is posted 20 lines above
	if (!wasDisabled) {
		printf("Killing Autonomous Thread\n");
		m_log << "Disabling Autonomous at time " << m_robot->threadsafeTime() << endl << endl;
		m_log.flush();
		// TODO(aschuh): Lock the csvreader first.
		m_robot->Lock();
		m_task->Stop();
		m_robot->Unlock();
	}
}

void MainRobot::PrintLogPos(const char* msg)
{
	//key:
	//1) Time (seconds)
	//2) Assumed X Position (feet)
	//3) Assumed Y Position (feet)
	//4) Assumed Angle (degrees)
	//5) Message
	double time=m_robot->threadsafeTime();
	double x=MetersToFeet(m_robot->GetAssumedXPos());
	double y=MetersToFeet(m_robot->GetAssumedYPos());
	double angle=RadiansToDegrees(m_robot->GetAssumedTheta());
	m_autolog << time;
	m_autolog << ", ";
	m_autolog << x;
	m_autolog << ", ";
	m_autolog << y;
	m_autolog << ", ";
	m_autolog << angle;
	m_autolog << ", ";
	m_autolog << msg;
	m_autolog << endl;
	m_autolog.flush();
	printf("%f x: %f y: %f angle: %f %s\n", time, x, y, angle, msg);
}

void MainRobot::LogRobotState()
{
	//key:
	//1) Time (seconds)
	//2) Left Encoder Distance (meters)
	//3) Right Encoder Distance (meters)
	//4) Gyro Value (radians)
	//5) Arm Angle (radians)
	//6) Elevator Encoder Height (meters)
	//7) High Gear Enabled (boolean)
	//8) Assumed X Position (meters)
	//9) Assumed Y Position (meters)
	//10) Assumed Angle (radians)
	m_robot->Lock();
	m_log << m_robot->GetTime();
	m_log << ", ";
	m_log << m_robot->GetLeftDistance();
	m_log << ", ";
	m_log << m_robot->GetRightDistance();
	m_log << ", ";
	m_log << m_robot->GetGyroValue();
	m_log << ", ";
	m_log << m_robot->GetArmAngle();
	m_log << ", ";
	m_log << m_robot->GetElevatorHeightValue();
	m_log << ", ";
	m_log << m_robot->isHighGear;
	m_log << ", ";
	m_log << m_robot->assumed_xpos;
	m_log << ", ";
	m_log << m_robot->assumed_ypos;
	m_log << ", ";
	m_log << m_robot->assumed_theta;
	m_log << endl;
	m_log.flush();
	m_robot->Unlock();
}

void MainRobot::OneTubeAutonomous() {
	//compensation for turns for scoring on the left or the right side
	int turn_compensation_factor = m_isScoringLeft ? 1 : -1;
	
	PrintLogPos("Starting One Tube Autonomous");
	m_lcd->PrintfLine(DriverStationLCD::kUser_Line1, "Auto: %05.2f: Initializing",m_robot->threadsafeTime());
	
	//make sure we're in high gear and the gyro is zeroed
	//this should already happen but just to make sure
	m_robot->Lock();
	m_robot->isHighGear = true;
	m_robot->ResetGyro();
	m_robot->Unlock();
	
	//don't change the arm angle - shouldn't matter, but makes
	//it less jerky in a fail situation
	m_robot->armAngleDegrees(RadiansToDegrees(m_robot->GetArmAngle()));
	
	//close the claw, roll the rollers in, and wait for the tube
	m_robot->closeClaw();
	m_robot->grabTube();
	m_robot->waitForTube();
	printf("%f: Tube in Claw\n", m_robot->threadsafeTime());
	
	//going to the rack
	//setting the arm up if it isn't already
	printf("%f: Going to the rack\n", m_robot->threadsafeTime());
	m_lcd->PrintfLine(DriverStationLCD::kUser_Line1, "Auto: %05.2f: Moving",m_robot->threadsafeTime());
	
	m_robot->driveForwardsFeet(m_csvReader->GetValue("DISTANCE_TO_WALL_FEET"));
	m_robot->armAngleDegrees(m_csvReader->GetValue("ARM_UP_ANGLE"));
	
	// Lift the elevator part way
	PrintLogPos("Raising the Elevator part way through");
	m_lcd->PrintfLine(DriverStationLCD::kUser_Line4, "Auto: %05.2f: Raising Elevator",m_robot->threadsafeTime());
	
	m_robot->waitForDriveFeetLeft(m_csvReader->GetValue("ONE_TUBE_WAIT_FEET"));
	m_robot->elevatorHeight(m_csvReader->GetValue("ELEVATOR_TOP_SIDE") + InchesToMeters(2.0));
	
	//wait for us to be there and the elevator to be up
	m_robot->waitForElevator();
	m_robot->armAngleDegrees(m_csvReader->GetValue("ARM_SCORE_ANGLE"));
	m_robot->waitForDrive();
	PrintLogPos("At the Goal");
	m_lcd->PrintfLine(DriverStationLCD::kUser_Line4, "Auto: %05.2f: At the Goal",m_robot->threadsafeTime());
	
	
	// Move over a tiny bit to avoid hitting the other bot.
	m_robot->turnAngleDegrees(turn_compensation_factor*m_csvReader->GetValue("AUTO_BACKUP_ANGLE_DEGREES"));
	
	// Lower the arm
	m_robot->armAngleDegrees(m_csvReader->GetValue("AUTO_ARM_DOWN_ANGLE_DEGREES"));
	
	// Place
	m_robot->openClaw();
	m_robot->waitForArmWithTimeout(m_csvReader->GetValue("ONE_TUBE_ARM_TIMEOUT_SECONDS"));
	
	
	// Lower elevator
	PrintLogPos("Claw Opened");
	m_lcd->PrintfLine(DriverStationLCD::kUser_Line4, "Auto: %05.2f: Claw Opened",m_robot->threadsafeTime());
	m_lcd->PrintfLine(DriverStationLCD::kUser_Line5, "Auto: %05.2f: One Tube Time",m_robot->threadsafeTime());
	
	m_robot->elevatorHeight(m_csvReader->GetValue("ELEVATOR_TOP_SIDE")-m_csvReader->GetValue("ONE_TUBE_ELEVATOR_LOWER_METERS"));
	m_robot->waitForElevator();
	
	// Stow arm and book it out of there, resetting the bot.
	m_robot->armAngleDegrees(m_csvReader->GetValue("ARM_UP_ANGLE"));
	//m_robot->driveForwardsFeet(-(m_csvReader->GetValue("DISTANCE_TO_WALL_FEET")));	
	PrintLogPos("Driving Backwards");
	m_lcd->PrintfLine(DriverStationLCD::kUser_Line4, "Auto: %05.2f: Driving Backwards",m_robot->threadsafeTime());
	
	
	//TODO(ebakan): see if some of these redundant statements can be
	//eliminated

	//back up, close the claw, zero the elevator
	m_robot->driveForwardsFeet(-(m_csvReader->GetValue("DISTANCE_TO_WALL_FEET")-m_csvReader->GetValue("AUTO_BACKUP_OFFSET_FEET")));	
	m_robot->closeClaw();
	m_robot->waitForArm();
	m_robot->elevatorHeight(m_csvReader->GetValue("ELEVATOR_GROUND"));
	m_robot->waitForDrive();
	m_robot->elevatorHeight(m_csvReader->GetValue("ELEVATOR_GROUND"));
	m_robot->waitForElevator();
	
	// Get ready for the next one.
	m_robot->ignoreTube();
	m_robot->armAngleDegrees(m_csvReader->GetValue("ARM_DOWN_ANGLE"));
	m_robot->elevatorHeight(m_csvReader->GetValue("ELEVATOR_GROUND"));
	m_robot->waitForElevator();
	
	PrintLogPos("One Tube Done");
	m_lcd->PrintfLine(DriverStationLCD::kUser_Line4, "Auto: %05.2f: One Tube Done",m_robot->threadsafeTime());
	
}

void MainRobot::TwoTubeAutonomous() {
	//similar compensation factor as above
	int turn_compensation_factor = m_isScoringLeft ? 1 : -1;

	//execute the one tube autonomous first
	OneTubeAutonomous();
	
	PrintLogPos("Initiating two tube autonomous");
	m_lcd->PrintfLine(DriverStationLCD::kUser_Line4, "Auto: %05.2f: Initiating Two Tube",m_robot->threadsafeTime());
	
	// Close the claw.  Don't suck in until aimed at the tube so you grab it centered.
	m_robot->closeClaw();
	
	// turn towards the tube.
	PrintLogPos("Turning 90 Degrees");
	m_lcd->PrintfLine(DriverStationLCD::kUser_Line4, "Auto: %05.2f: Turning 90 degrees",m_robot->threadsafeTime());
	
	m_robot->turnAngleDegrees(turn_compensation_factor*m_csvReader->GetValue("TWO_TUBE_TUBE_TURN_ANGLE_DEGREES"));
	m_robot->waitForDriveWithinDegrees(2.0);
	Wait(0.1);
	m_robot->waitForDriveWithinDegrees(2.0);
	m_robot->grabTube();
	PrintLogPos("Done Turning 90 Degrees");
	m_lcd->PrintfLine(DriverStationLCD::kUser_Line4, "Auto: %05.2f: Turn Done",m_robot->threadsafeTime());
	
	
	// drive forwards and then wait until drive is
	// done and tube is grabbed
	m_robot->driveForwardsFeet(m_csvReader->GetValue("TWO_TUBE_TUBE_GRAB_DISTANCE_FEET"));
	int tubeCount = 0;
	int numTubeCountsToCheck=5;
	while (tubeCount<numTubeCountsToCheck) {
		if(m_robot->hasTube())
			tubeCount++;
		else
			tubeCount=0;
		Wait(m_robot->dt);
	}
	m_robot->armAngleDegrees(m_csvReader->GetValue("ARM_UP_ANGLE"));
	
	PrintLogPos("Done driving forwards and got tube");
	m_lcd->PrintfLine(DriverStationLCD::kUser_Line4, "Auto: %05.2f: Got Tube",m_robot->threadsafeTime());
	
	//backing up to original position
	m_robot->driveForwardsFeet(-m_csvReader->GetValue("TWO_TUBE_TUBE_GRAB_DISTANCE_FEET"));
	m_robot->waitForDrive();
	PrintLogPos("Done backing up");
	m_lcd->PrintfLine(DriverStationLCD::kUser_Line4, "Auto: %05.2f: Backed Up",m_robot->threadsafeTime());
	
	
	// Turn towards the peg from facing the tube.
	// Working number.  Hits side of peg slightly.
	m_robot->turnAngleDegrees(-turn_compensation_factor*m_csvReader->GetValue("TWO_TUBE_PEG_ANGLE_DEGREES"));
	m_robot->waitForDriveWithinDegrees(1.3);
	PrintLogPos("Done turning back");
	// Make sure this isn't an overshoot.  That would be really bad.
	Wait(0.2);
	m_robot->waitForDriveWithinDegrees(1.3);
	PrintLogPos("Done turning back really");
	m_lcd->PrintfLine(DriverStationLCD::kUser_Line4, "Auto: %05.2f: Lined Up",m_robot->threadsafeTime());
	
	
	//drive towards the peg
	m_robot->driveForwardsFeet(m_csvReader->GetValue("TWO_TUBE_PEG_DISTANCE_FEET"));
	m_robot->waitForDriveFeetLeft(m_csvReader->GetValue("TWO_TUBE_WAIT_FEET"));
	PrintLogPos("Almost there");
	m_lcd->PrintfLine(DriverStationLCD::kUser_Line4, "Auto: %05.2f: Raising Elevator",m_robot->threadsafeTime());
	
	
	//m_robot->elevatorHeight(m_csvReader->GetValue("ELEVATOR_TOP_MIDDLE"));
	m_robot->elevatorHeight(m_csvReader->GetValue("ELEVATOR_TOP_SIDE") + InchesToMeters(2.0));
	
	// Get there and settle down.
	m_robot->waitForElevator();
	m_robot->armAngleDegrees(m_csvReader->GetValue("ARM_SCORE_ANGLE"));
	
	m_robot->waitForDrive();
	// TODO(aschuh): Add the rest back in when it works.
	PrintLogPos("Done Driving Auto Mode");
	m_lcd->PrintfLine(DriverStationLCD::kUser_Line4, "Auto: %05.2f: Ready At Goal",m_robot->threadsafeTime());
	
	
	// Lower the arm
	m_robot->armAngleDegrees(m_csvReader->GetValue("AUTO_ARM_DOWN_ANGLE_DEGREES"));
	m_robot->waitForArmWithTimeout(m_csvReader->GetValue("TWO_TUBE_ARM_TIMEOUT_SECONDS"));
	PrintLogPos("Arm Down");
	
	// Place
	m_robot->openClaw();
	m_lcd->PrintfLine(DriverStationLCD::kUser_Line4, "Auto: %05.2f: Tube Placed",m_robot->threadsafeTime());
	m_lcd->PrintfLine(DriverStationLCD::kUser_Line5, "Auto: %05.2f: Two Tube Time",m_robot->threadsafeTime());
	
	
	// Lower elevator
	m_robot->elevatorHeight(m_csvReader->GetValue("ELEVATOR_TOP_MIDDLE")-m_csvReader->GetValue("TWO_TUBE_ELEVATOR_LOWER_METERS"));
	m_robot->waitForElevator();
	PrintLogPos("Elevator Placed");
	
	// Stow arm and book it out of there, resetting the bot.
	m_robot->armAngleDegrees(m_csvReader->GetValue("ARM_UP_ANGLE"));
	//m_robot->driveForwardsFeet(-(m_csvReader->GetValue("DISTANCE_TO_WALL_FEET")+0));	
	m_robot->driveForwardsFeet(-m_csvReader->GetValue("TWO_TUBE_BACKUP_FEET"));
	m_robot->closeClaw();
	m_robot->waitForArm();
	PrintLogPos("Arm Stowed");
	m_robot->elevatorHeight(m_csvReader->GetValue("ELEVATOR_GROUND"));
	m_robot->waitForDrive();
	PrintLogPos("Drive Done");
	m_robot->waitForElevator();
	PrintLogPos("Stopped Moving.  Auto mode officially done");
	m_lcd->PrintfLine(DriverStationLCD::kUser_Line4, "Auto: %05.2f: Auto Mode Done",m_robot->threadsafeTime());
	
}	

void MainRobot::RunAutoThread() {

	//executes a different autonomous based on the current user
	//selection
	if(m_robot->currAutonomous==RobotState::kOneTube)
			OneTubeAutonomous();
		else if(m_robot->currAutonomous==RobotState::kTwoTubeLeft)
		{
			m_isScoringLeft=true;
			TwoTubeAutonomous();
		}
		else if(m_robot->currAutonomous==RobotState::kTwoTubeRight)
		{
			m_isScoringLeft=false;
			TwoTubeAutonomous();
		}
		else {
			m_lcd->Printf(DriverStationLCD::kUser_Line1,1,"No Autonomous Selected");
		}
}

void MainRobot::OperatorControl()
{
	// TODO(aschuh): Provide reset hooks so that things can be reset.
	m_log << "Entering Operator Control" << endl;
	m_log.flush();
	GetWatchdog().SetEnabled(true);
	m_csvReader->ReloadValues(); 
	m_robot->ResetState();
	m_controlLoops->Reset();
	m_robot->isOperatorControl=true;
	m_robot->isAutonomous=false;
	printf("Op Control\n");
	bool disabledState = IsDisabled();
	while (IsOperatorControl()) {
		m_robot->isDisabled = IsDisabled();
		// enabled
		if(!IsDisabled()) {
			LogRobotState();
		}			
		//disabled

		//only updated when a new packet is received from the
		//control board
		if(m_ds->GetPacketNumber() != m_prevPacketNumber) {
			m_prevPacketNumber=m_ds->GetPacketNumber();
			if (disabledState == true && IsDisabled() == false) {
				m_controlBoard->ResetState();
				m_csvReader->ReloadValues();
				m_robot->ResetState();
				m_robot->isOperatorControl=true;
				m_robot->isAutonomous=false;
				m_controlLoops->Reset();
				m_log << "Enabling Teleop at time " << m_robot->threadsafeTime() << endl;
			} else if (disabledState == false && IsDisabled() == true) {
				m_log << "Disabling Teleop at time " << m_robot->threadsafeTime() << endl << endl;
			}
			disabledState = IsDisabled(); 
			m_controlBoard->UpdateValues();
		}
		//printf("Feeding...\n");
		GetWatchdog().Feed();
		Wait(0.001);
	}
	printf("Done with operator control\n");
}

//start the actual program
START_ROBOT_CLASS(MainRobot);

