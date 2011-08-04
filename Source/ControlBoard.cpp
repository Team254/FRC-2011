#include "ControlBoard.h"
#include "RobotState.h"
#include "CSVReader.h"
#include "CommonIncludes.hpp"
#include "ControlBoardPorts.h"

ControlBoard::ControlBoard(RobotState* robotState, CSVReader* csvReader)
: m_robot(robotState)
, m_csvReader(csvReader)
, m_LeftJoystick(1)
, m_RightJoystick(2)
, m_ESTOPBoard(3)
, m_initialMinibotRelease(getBoardButton(kMinibotReleaseButton))
, m_initialMinibotDeploy(getBoardButton(kMinibotDeployButton))
, m_initialAutoScoreStatus(getBoardButton(kScoreSwitch))
, m_autoScoreFlipped(false)
, m_arm_is_set_up(false)
, m_elevator_set_up(false)
, m_rollout_time(0.0)
, m_low_peg(false)
, m_prevArmPower(0.0)
, m_prevElevatorJoystick(0.0)
, m_elevator_destination(0.0)
, m_elevator_decrement(0.0)
, m_autoScoreState(lowering_elevator)
{}

void ControlBoard::ResetState()
{
	printf("resetting control board\n");
	CancelSemiAutoActions();
	ResetToggleSwitches();
}


void ControlBoard::CancelSemiAutoActions()
{
	// preset height vars
	m_robot->isSetpointing=false;
	m_arm_is_set_up = false;
	m_elevator_set_up = false;
	m_robot->elevatorEnabled=true;

	// autoscore vars
	m_autoScoreFlipped=false;
	m_autoScoreState=lowering_elevator;
	m_rollout_time = 0.0;
	m_low_peg = false;
	m_elevator_destination=0.0;
	m_elevator_decrement=0.0;
	m_robot->grabberOpen = false;

	// other vars
	m_robot->isGrounding=false;
	m_robot->isControlLoopDriving=false;
}

void ControlBoard::ResetToggleSwitches() {
    m_initialMinibotRelease=getBoardButton(kMinibotReleaseButton);
    m_initialMinibotDeploy=getBoardButton(kMinibotDeployButton);
}

void ControlBoard::UpdateValues()
{
	m_robot->Lock();
	//reset these lines so they don't print XXX disabled when enabled
    m_robot->m_lcd->PrintfLine(DriverStationLCD::kUser_Line4, "");
    m_robot->m_lcd->PrintfLine(DriverStationLCD::kUser_Line5, "");
	//auton selection
        if(m_robot->isDisabled)
        {
                m_robot->Lock();
                if(getBoardButton(kSingleSelect))
                {
                        m_robot->currAutonomous=RobotState::kOneTube;
                        m_robot->autonomousDescription = "Auto: One Tube";
                }
                else if (getBoardButton(kDoubleLeftSelect))
                {
                        m_robot->currAutonomous=RobotState::kTwoTubeLeft;
                        m_robot->autonomousDescription = "Auto: Two Tube Left";
                }
                else if(getBoardButton(kDoubleRightSelect))
                {
                        m_robot->currAutonomous=RobotState::kTwoTubeRight;
                        m_robot->autonomousDescription = "Auto: Two Tube Right";
                }
                else if(getBoardButton(kLowButton))
                {
                        m_robot->currAutonomous=RobotState::kNoTube;
                        m_robot->autonomousDescription = "Auto: None";
                }
                else
                {
                        if(m_robot->currAutonomous == RobotState::kNoTube)
                        {
                                m_robot->autonomousDescription = "Auto: None";
                        }
                }
                m_robot->m_lcd->PrintfLine(DriverStationLCD::kUser_Line1, m_robot->autonomousDescription);

		//reset the minibot toggle switches and display the status on the LCD
		//NOTE: these will always be disabled, but it was included just to confirm
		//their status
                ResetToggleSwitches();
                if(ReleaseMinibot())
                    m_robot->m_lcd->PrintfLine(DriverStationLCD::kUser_Line4, "Releasing Enabled");
                else
                    m_robot->m_lcd->PrintfLine(DriverStationLCD::kUser_Line4, "Releasing Disabled");
                
                if(DeployMinibot())
                    m_robot->m_lcd->PrintfLine(DriverStationLCD::kUser_Line5, "Deploying Enabled");
                else
                    m_robot->m_lcd->PrintfLine(DriverStationLCD::kUser_Line5, "Deploying Disabled");
                //m_robot->m_lcd->UpdateLCD();
                	
                m_robot->Unlock();
        }        
	
	//printf("Got Lock cb\n");
	//tests
	//printf("time: %f\n",m_robot->GetTime());
    /*
    if(getBoardButton(kOperatorScore)) {
    	m_robot->ResetGyro();
    	m_robot->ResetLeftEncoder();
    	m_robot->ResetRightEncoder();
    }
	*/
        
	// set all the booleans for other control loops

	// toggle switches
    m_robot->minibotRelease=ReleaseMinibot();
	m_robot->minibotDeploy=DeployMinibot();
	m_robot->controlLoopsOn=ControlLoopsOn();
	
	// drivetrain
	m_robot->straightDrivePower=GetStraightDrivePower();
	m_robot->turnPower=GetTurnPower();
	//printf("Turn power is %f\n", m_robot->turnPower);
	m_robot->isQuickTurn=WantQuickTurn();
	m_robot->isHighGear=WantHighGear();
	//printf("High Gear: %d\n", m_robot->isHighGear);
	
	// elevator
	m_robot->elevatorPower=GetElevatorPower();
	SetElevatorHeight();
	
	// arm stow&ground presets
	// if we're disabled make it just maintain its current position
	if (getBoardButton(kStow)) {
		CancelSemiAutoActions();
		if(m_robot->isDisabled) {
			m_robot->armGoal = m_robot->GetArmAngle();
		}
		else {
			m_robot->armGoal = DegreesToRadians(m_csvReader->GetValue("ARM_UP_ANGLE"));
			m_robot->elevatorGoal = 0.0;
		}
		
	} else if (getBoardButton(kGroundPickup)) {
		CancelSemiAutoActions();
		if(m_robot->isDisabled) {
			m_robot->armGoal = m_robot->GetArmAngle();
		}
		else {
			m_robot->isGrounding = true;
			m_robot->elevatorGoal = 0.0;
		}
	}
	
	// rollers
	m_robot->rollerSpinVal = GetRollerSpinVal();
	
	// grounding
	if (m_robot->isGrounding) {
		m_robot->elevatorEnabled=true;
		if (GetArmPower() != 0 || hasElevatorUserInput()) {
			CancelSemiAutoActions();
		} else if (m_robot->elevatorStopped()) {
			CancelSemiAutoActions();
			m_robot->armGoal = DegreesToRadians(m_csvReader->GetValue("ARM_DOWN_ANGLE"));
		}
	} else if (m_robot->isSetpointing) {
		// setpointing

		if (GetArmPower() != 0) {
			CancelSemiAutoActions();
		}
		else {
			// we need to put the arm upright first
			//m_robot->armGoal = DegreesToRadians(m_csvReader->GetValue("ARM_UP_ANGLE"));
			if(!m_arm_is_set_up) {
				printf("setting up arm\n");
				// first store where the elevator wants to go, then wait for the arm to be finished
				m_robot->elevatorEnabled=false;
				m_robot->armGoal = DegreesToRadians(m_csvReader->GetValue("ARM_UP_ANGLE"));
				// the arm is now set up, but the elevator is not
				m_arm_is_set_up=true;
				m_elevator_set_up=false;
			}
			
			// once the arm is up and the elevator hasn't been set up, set up the elevator to its desired height
			else if(m_robot->armStopped() && !m_elevator_set_up) {
				printf("setting up elevator\n");
				m_robot->elevatorEnabled=true;
				m_elevator_set_up=true;
			}
			// once the elevator is set up and done going to its height, set the arm to the right angle
			if(m_elevator_set_up && m_robot->elevatorStopped()) {
				printf("setting up arm goal\n");
				m_robot->armGoal = DegreesToRadians(m_csvReader->GetValue("ARM_SCORE_ANGLE"));
				// everything's done - reset the static bools
				if(m_robot->armStopped()) {
					CancelSemiAutoActions();
				}
			}
		}
	}
	else //safety - make sure the elevator is enabled
		m_robot->elevatorEnabled=true;
	
	//autoscore if needed
	if(AutoScoreOn()){
		AutoScore();
	}
	
	m_robot->armPower=GetArmPower();

	//give elevator and arm control a little bit of drift
	double move_angle = DegreesToRadians(m_csvReader->GetValue("ARM_MOVE_SPEED"));
	double max_up = DegreesToRadians(m_csvReader->GetValue("ARM_UP_ANGLE"));
	double max_down = DegreesToRadians(m_csvReader->GetValue("ARM_DOWN_ANGLE"));
	if (m_robot->armPower > 0.1) {
		m_robot->armGoal += move_angle;
	} else if (m_robot->armPower < -0.1) {
		m_robot->armGoal -= move_angle;
	} else if(fabs(m_prevArmPower)>.1){
		m_robot->armGoal = m_robot->GetArmAngle() + .08 * m_robot->armVelocity;
	}
	
	//cap the max arm goal
	if (m_robot->armGoal <= max_down)
		m_robot->armGoal = max_down;
	if (m_robot->armGoal >= max_up)
		m_robot->armGoal = max_up;
	
	m_prevArmPower = m_robot->armPower;

	//convert buttons of the control loops into desired functions
	if (m_robot->controlLoopsOn) {
		if (m_robot->isRollingIn) {
			if (WantRollOut()) {
				m_robot->isRollingIn=false;
				m_robot->isRollingOut=true;
			}
		} else {
			if (WantRollIn()) {
				m_robot->isRollingIn=true;
				m_robot->isRollingOut=false;
			} else {
				m_robot->isRollingOut=WantRollOut();
			}
		}
	} else {
		m_robot->isRollingOut=WantRollOut();
		m_robot->isRollingIn=WantRollIn();
	}
	
	//printf("Release Lock cb\n");
	m_robot->Unlock();
}

// drivetrain
double ControlBoard::GetStraightDrivePower()
{
	return getJoystickAxisValue(kLeftJoystickYAxis);
}

double ControlBoard::GetTurnPower()
{
	return getJoystickAxisValue(kRightJoystickXAxis);
}

bool ControlBoard::WantQuickTurn()
{
	return getJoystickButton(kRightJoystickButton);
}

bool ControlBoard::WantHighGear()
{
	return !getJoystickButton(kLeftJoystickSwitch);
}

// elevator
double ControlBoard::GetElevatorPower()
{
	return getJoystickAxisValue(kElevatorJoystickAxis);
}

bool ControlBoard::hasElevatorUserInput()
{
	return getBoardButton(kLowButton) ||
			getBoardButton(kLowUpButton) ||
			getBoardButton(kMiddleButton) ||
			getBoardButton(kMiddleUpButton) ||
			getBoardButton(kTopButton) ||
			getBoardButton(kTopUpButton) ||
			getJoystickAxisValue(kElevatorJoystickAxis) != 0.0;
}

void ControlBoard::SetElevatorHeight()
{
	//control elevator preset buttons
	if (getBoardButton(kLowButton)) {
		CancelSemiAutoActions();
		m_robot->elevatorGoal = m_csvReader->GetValue("ELEVATOR_LOW_SIDE");
		m_robot->isZeroingElevator = true;
		m_robot->isSetpointing = true;
	} else if (getBoardButton(kLowUpButton)) {
		CancelSemiAutoActions();
		m_robot->elevatorGoal = m_csvReader->GetValue("ELEVATOR_LOW_MIDDLE");
		m_robot->isSetpointing = true;
	} else if (getBoardButton(kMiddleButton)) {
		CancelSemiAutoActions();
		m_robot->elevatorGoal = m_csvReader->GetValue("ELEVATOR_MIDDLE_SIDE");
		m_robot->isSetpointing = true;
	} else if (getBoardButton(kMiddleUpButton)) {
		CancelSemiAutoActions();
		m_robot->elevatorGoal = m_csvReader->GetValue("ELEVATOR_MIDDLE_MIDDLE");
		m_robot->isSetpointing = true;
	} else if (getBoardButton(kTopButton)) {
		CancelSemiAutoActions();
		m_robot->elevatorGoal = m_csvReader->GetValue("ELEVATOR_TOP_SIDE");
		m_robot->isSetpointing = true;
	} else if (getBoardButton(kTopUpButton)) {
		CancelSemiAutoActions();
		m_robot->elevatorGoal = m_csvReader->GetValue("ELEVATOR_TOP_MIDDLE");
		m_robot->isSetpointing = true;
	} else {
		// TODO(aschuh): Add in the slowing down.
		double elevatorJoystick = getJoystickAxisValue(kElevatorJoystickAxis);
		double max_up = m_csvReader->GetValue("ELEVATOR_UP_MAX_SPEED");
		double max_down = fabs(m_csvReader->GetValue("ELEVATOR_DOWN_MAX_SPEED"));
		//printf("Elevator Joystick is %f\n", elevatorJoystick);
		//overriding presets with the joystick
		if (elevatorJoystick > .1) {
			CancelSemiAutoActions();
			m_robot->isSetpointing = false;
			m_robot->isZeroingElevator = false;
			m_robot->elevatorGoal += elevatorJoystick * max_up;
		} else if (elevatorJoystick < -.1) {
			CancelSemiAutoActions();
			m_robot->isSetpointing = false;
			m_robot->isZeroingElevator = false;
			m_robot->elevatorGoal += elevatorJoystick * max_down;
			if (m_robot->elevatorGoal < 0.0) {
				m_robot->isZeroingElevator = true;
			}
		} else if (fabs(m_prevElevatorJoystick) > .1) {
			m_robot->elevatorGoal = m_robot->GetElevatorHeightValue();
			// If it runs immediately like this it jumps down to correct from letting it go, Sides likes it to
			// smooth in rather than immediately stop
			if (m_prevElevatorJoystick > .1)
				m_robot->elevatorGoal += .25 * m_robot->elevatorVelocity;
			else
				m_robot->elevatorGoal += .15 * m_robot->elevatorVelocity;
		}
		m_prevElevatorJoystick = elevatorJoystick;
	}
	double maxHeight = m_csvReader->GetValue("ELEVATOR_UP_HEIGHT_FEET");
	if (m_robot->elevatorGoal >= maxHeight) {
		m_robot->elevatorGoal= maxHeight;
	} else if (m_robot->elevatorGoal <= 0.0) {
		m_robot->elevatorGoal = 0.0;
	}
}

// arm
double ControlBoard::GetArmPower()
{
	if (getBoardButton(kArmUpButton)) {
		return 1;
	} else if (getBoardButton(kArmDownButton)) {
		return -1;
	} else {
		return 0;
	}
}

// rollers
double ControlBoard::GetRollerSpinVal()
{
	return getJoystickAxisValue(kRollerJoystickAxis);
}

bool ControlBoard::WantRollIn()
{
	return getJoystickButton(kLeftJoystickButton) || getBoardButton(kRollInButton);
}

bool ControlBoard::WantRollOut()
{
	return getBoardButton(kRollOutButton);
}

// other
bool ControlBoard::ReleaseMinibot()
{
	return m_initialMinibotRelease != getBoardButton(kMinibotReleaseButton);
}

bool ControlBoard::DeployMinibot()
{
	return m_initialMinibotDeploy != getBoardButton(kMinibotDeployButton);
}

bool ControlBoard::ControlLoopsOn()
{
	return !getBoardButton(kSensorCancelButton);
}

bool ControlBoard::AutoScoreOn()
{
	if(getBoardButton(kScoreSwitch) != m_initialAutoScoreStatus)
	{
		CancelSemiAutoActions();
		m_initialAutoScoreStatus = getBoardButton(kScoreSwitch);
		m_autoScoreFlipped = !m_autoScoreFlipped;
	}
	return m_autoScoreFlipped;
}

void ControlBoard::AutoScore()
{
	// if a board button is hit, stop the action
	if(//arm button hit
	   getBoardButton(kArmUpButton) ||
	   getBoardButton(kArmDownButton) ||
	   //elevator button hit
	   hasElevatorUserInput() ||
	   //grounding button hit
	   getBoardButton(kStow) ||
	   getBoardButton(kGroundPickup)) {
		CancelSemiAutoActions();
		return;
	}
	
	// always set the right arm angle
	if(m_low_peg)
		m_robot->armGoal = DegreesToRadians(m_csvReader->GetValue("ARM_DOWN_ANGLE"));
	else
		m_robot->armGoal = DegreesToRadians(m_csvReader->GetValue("ARM_SCORE_ANGLE"));
	//switch
	switch (m_autoScoreState) {
			// lowering the elevator, setting everything up
		case lowering_elevator:
			m_elevator_destination = m_robot->GetElevatorHeightValue()-m_csvReader->GetValue("AUTO_SCORE_ELEVATOR_DOWN_FIRST_FEET");
			m_elevator_decrement = m_csvReader->GetValue("AUTO_SCORE_ELEVATOR_DOWN_FIRST_FEET") * m_robot->dt * 2.0 / 0.25;
			// if we're going to bottom out, switch to the low peg
			if (m_elevator_destination - .4 <= 0) {
                m_low_peg=true;
                m_rollout_time=.5;
			} else {
                m_low_peg=false;
                m_rollout_time=0.0;
			}
			m_autoScoreState = decrementing_elevator_height_first;
			break;
			
			//decrement the goal until the final destination is reached
		case decrementing_elevator_height_first:
			//we're not there yet or we're not at the bottom
			if (fabs(m_elevator_destination - m_robot->elevatorGoal) >= InchesToMeters(1.0) && m_robot->GetElevatorHeightValue()>=InchesToMeters(1.0)) {
				m_robot->elevatorGoal -= m_elevator_decrement;
			} else {
				//go on to the next state
				m_autoScoreState = lowering_elevator_second;
			}
			break;
			
			//open the grabber and lower the elevator the second amount
		case lowering_elevator_second:
			m_elevator_destination = m_robot->GetElevatorHeightValue() - m_csvReader->GetValue("AUTO_SCORE_ELEVATOR_DOWN_SECOND_FEET");
			m_elevator_decrement = m_csvReader->GetValue("AUTO_SCORE_ELEVATOR_DOWN_SECOND_FEET") * m_robot->dt * 2.0 / 0.25;
			m_robot->grabberOpen = true;
			m_autoScoreState = decrementing_elevator_height_second;
			break;
			
			//decrement the goal until the final destination is reached
		case decrementing_elevator_height_second:
			//we're not there yet
			m_robot->openClaw();
			//m_robot->ejectTube();
			if (fabs(m_elevator_destination-m_robot->elevatorGoal)>= InchesToMeters(1.0) && m_robot->GetElevatorHeightValue()>=InchesToMeters(1.0)) {
				m_robot->elevatorGoal -= m_elevator_decrement;
				printf("Lowering slowly\n");
			} else if(m_robot->armStopped() && m_rollout_time <= 0){
				//wait until the arm is stopped and the rollers are done
				m_autoScoreState = backing_away;
			}
			break;
			
			//backing away, resetting everything
		case backing_away:
			printf("resetting stuff\n");
			m_robot->ResetLeftEncoder();
			m_robot->ResetRightEncoder();
			m_robot->ResetGyro();
			m_robot->straightDistanceGoal = 0.0;
			m_robot->turnAngleGoal=0.0;
			m_robot->resetDriveControl = true;
			m_robot->straightDistanceMaxVelocity = 3.0;
			m_robot->straightDistanceMaxAcceleration = 8.0;
			m_robot->driveForwardsFeet(-m_csvReader->GetValue("AUTO_SCORE_BACKUP_FEET"));
			//note: this skipping raising_arm is intentional
			m_autoScoreState=waiting_to_complete;
			break;
			
		case waiting_to_complete:
			//if we've either stopped our drive or we've overridden the drive
			//by moving the control sticks then cancel errythang and move the arm up
			if (m_robot->driveStopped() || !m_robot->isControlLoopDriving) {
				m_robot->ignoreTube();
				m_robot->closeClaw();
				CancelSemiAutoActions();
				m_robot->armGoal = DegreesToRadians(m_csvReader->GetValue("ARM_UP_ANGLE"));
			}
			break;
		// default resets everything
		default:
			printf("resetting\n");
			CancelSemiAutoActions();
			m_robot->ignoreTube();
			m_robot->closeClaw();
			break;
	}
	// handle the rollouts
	if (m_rollout_time > 0.0)
		m_rollout_time -= m_robot->dt; //compensate for control board getting updated infrequently
}

// helper functions
bool ControlBoard::getJoystickButton(JoystickButton button)
{
	switch (button) {
		case kLeftJoystickButton:
			return m_LeftJoystick.GetRawButton(4);
		case kLeftJoystickSwitch:
			return m_LeftJoystick.GetRawButton(2);
		case kLeftJoystickTrigger:
			return m_LeftJoystick.GetRawButton(1);
		case kRightJoystickButton:
			return m_RightJoystick.GetRawButton(4);
		case kRightJoystickSwitch:
			return m_RightJoystick.GetRawButton(2);
		default:
			return false;
	}
}

bool ControlBoard::getBoardButton(ControlBoardButton button)
{
	//TODO:: fill in switch statement
	switch (button) {
		case kGroundPickup:
			return m_ESTOPBoard.GetRawButton(ESTOP_GROUND);
		case kStow:
			return m_ESTOPBoard.GetRawButton(ESTOP_STOW);
		case kOperatorScore:
			return m_ESTOPBoard.GetRawButton(ESTOP_SCORE);
		case kScoreSwitch:
			//TODO(Caldwell): Look at the proposed change to autoscore controls
			return getJoystickButton(kRightJoystickSwitch);
			//This would toggle whether the scoring is manual like before or uses
			//an auto score feature like we have in autonomous, just putting it in
			//here to avoid making other changes
			//return m_ESTOPBoard.GetRawButton(ESTOP_SCORE);
		case kScoreButton:
			return m_ESTOPBoard.GetRawButton(ESTOP_SCORE);
		case kMinibotReleaseButton:
			return m_ESTOPBoard.GetRawButton(ESTOP_MINIBOT_RELEASE);
		case kMinibotDeployButton:
			return m_ESTOPBoard.GetRawButton(ESTOP_MINIBOT_DEPLOY);
		case kArmUpButton:
			return CalculateArmButton(kArmUpButton);
		case kArmDownButton:
			return CalculateArmButton(kArmDownButton);
		case kLowButton:
			return m_ESTOPBoard.GetRawButton(ESTOP_LOW_PORT);
		case kLowUpButton:
			return m_ESTOPBoard.GetRawButton(ESTOP_LOW_UP_PORT);
		case kMiddleButton:
			return m_ESTOPBoard.GetRawButton(ESTOP_MIDDLE_PORT);
		case kMiddleUpButton:
			return m_ESTOPBoard.GetRawButton(ESTOP_MIDDLE_UP_PORT);
		case kTopButton:
			return m_ESTOPBoard.GetRawButton(ESTOP_TOP_PORT);
		case kTopUpButton:
			return m_ESTOPBoard.GetRawButton(ESTOP_TOP_UP_PORT);
		case kRollInButton:
			return false;
		case kRollOutButton:
			return m_LeftJoystick.GetRawButton(1);
		case kSingleSelect:
			return m_ESTOPBoard.GetRawButton(ESTOP_ONETUBE);
		case kDoubleLeftSelect:
			return m_ESTOPBoard.GetRawButton(ESTOP_TWOTUBELEFT);
		case kDoubleRightSelect:
			return m_ESTOPBoard.GetRawButton(ESTOP_TWOTUBERIGHT);
		case kSensorCancelButton:
			return m_ESTOPBoard.GetRawButton(ESTOP_SENSOR_CANCEL_PORT);
		default:
			return false;
	}
	
}

bool ControlBoard::getLED(ControlBoardLEDButton led)
{
	switch (led) {
    	case kRedLEDButton:
    		return m_ESTOPBoard.GetRawButton(1);
    	case kBlueLEDButton:
    		return m_ESTOPBoard.GetRawButton(2);
    	case kWhiteLEDButton:
    		return m_ESTOPBoard.GetRawButton(3);
        default:
        	return false;
    }	
}

bool ControlBoard::CalculateArmButton(ControlBoardButton button)
{
	if(button == kArmUpButton && m_ESTOPBoard.GetRawAxis(ESTOP_ARM_UP_PORT) < -0.8 && m_ESTOPBoard.GetRawAxis(ESTOP_ARM_DOWN_PORT) > -0.8)
		return true;
	if(button == kArmDownButton && m_ESTOPBoard.GetRawAxis(ESTOP_ARM_DOWN_PORT) < -0.8)
		return true;
	return false;	
}

double ControlBoard::getJoystickAxisValue(JoystickAxis axis)
{
	switch (axis) {
        case kLeftJoystickYAxis:
            return handleDeadband(m_LeftJoystick.GetRawAxis(2), 0.1);
        case kRightJoystickXAxis:
        	return handleDeadband(m_RightJoystick.GetRawAxis(1), 0.1);
        case kElevatorJoystickAxis:
        	return handleDeadband(-m_ESTOPBoard.GetRawAxis(2), 0.10);
        case kRollerJoystickAxis:
        {
        	float rawAxisVal = handleDeadband(m_ESTOPBoard.GetRawAxis(1), .2);
        	rawAxisVal = (rawAxisVal < 0) ? rawAxisVal / 0.8 : rawAxisVal;
        	if (fabs(rawAxisVal) > 1.0)
        		rawAxisVal = signum(rawAxisVal) * 1.0;
        	return rawAxisVal;
        }
        default:
            return 0;
    }	
}

double ControlBoard::handleDeadband(double val, double deadband)
{
	return (fabs(val) > fabs(deadband)) ? val : 0.0;
}

