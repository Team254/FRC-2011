#include "ElevatorControl.h"
#include "CommonIncludes.hpp"

ElevatorControl::ElevatorControl(RobotState* robot, CSVReader* csvReader)
{
	m_robot = robot;
	m_csvReader = csvReader;
	Reset();
}

void ElevatorControl::Reset()
{
	m_last_elevator_height=0.0;
	m_elevator_integral=0.0;
}

void ElevatorControl::Update()
{
	m_robot->Lock();
	//printf("elevator: %f\n",m_robot->GetElevatorHeightValue());
	// control loop toggle
	if (!m_robot->controlLoopsOn)
	{
		m_robot->SetElevatorMotor(m_robot->elevatorPower);
		m_robot->Unlock();
		return;
	}	
	
	// checking if at hard stops
	double max_up = FeetToMeters(m_csvReader->GetValue("ELEVATOR_UP_HEIGHT_FEET"));
	double max_down = 0;
	if (m_robot->elevatorGoal <= max_down)
		m_robot->elevatorGoal = max_down;
	if (m_robot->elevatorGoal >= max_up)
		m_robot->elevatorGoal = max_up;		

	double elevator_height = m_robot->GetElevatorHeightValue();
	double height_error = m_robot->elevatorGoal - elevator_height;
	double height_deriv = elevator_height - m_last_elevator_height;
	m_robot->elevatorVelocity = height_deriv / m_robot->dt;
	m_elevator_integral += height_error;
	
	//printf("Elevator Error %f ", height_error);

	double kP;
	double kI;
	double kD;

	if (height_error >= 0 && m_robot->GetSecondStageLimitSwitch()) {
		kP = m_csvReader->GetValue("KP_ELEVATOR_UP_SECOND_DOWN");
		kI = m_csvReader->GetValue("KI_ELEVATOR_UP_SECOND_DOWN");
		kD = m_csvReader->GetValue("KD_ELEVATOR_UP_SECOND_DOWN");
	} else if (height_error > 0) {
		kP = m_csvReader->GetValue("KP_ELEVATOR_UP_SECOND_UP");
		kI = m_csvReader->GetValue("KI_ELEVATOR_UP_SECOND_UP");
		kD = m_csvReader->GetValue("KD_ELEVATOR_UP_SECOND_UP");
	} else if (height_error <= 0 && m_robot->GetSecondStageLimitSwitch()) {
		kP = m_csvReader->GetValue("KP_ELEVATOR_DOWN_SECOND_DOWN");
		kI = m_csvReader->GetValue("KI_ELEVATOR_DOWN_SECOND_DOWN");
		kD = m_csvReader->GetValue("KD_ELEVATOR_DOWN_SECOND_DOWN");
	} else {
		kP = m_csvReader->GetValue("KP_ELEVATOR_DOWN_SECOND_UP");
		kI = m_csvReader->GetValue("KI_ELEVATOR_DOWN_SECOND_UP");
		kD = m_csvReader->GetValue("KD_ELEVATOR_DOWN_SECOND_UP");
	}
	
	double power = kP * height_error + kD * height_deriv;
	
	if (kI * m_elevator_integral > 1)
		m_elevator_integral = 1/kI;
	else if (kI * m_elevator_integral < -1)
		m_elevator_integral = -1/kI;
	
	power += kI * m_elevator_integral;
	
	if (m_robot->isZeroingElevator) {
		// If the current height is close to the point where it would 
		// stop manually apply current to make it go down until it has
		// zeroed. Resets encoder if it thinks it is at negative height
		//printf("Trying to zero\n");
		if (elevator_height <= InchesToMeters(1) || power == 0) {
			//printf("Force driving down \n");
			power = -.2;
			if (elevator_height < 0)
				m_robot->ResetElevatorEncoder();
		}
		// Makes sure both stages are down, then resets the elevator encoderr
		// Added a check to make sure it's above 6 inches, otherwise the force from
		// the downward acceleration can trip the carriage limit switch
		if (m_robot->GetCarriageLimitSwitch() && m_robot->GetSecondStageLimitSwitch() && elevator_height<=InchesToMeters(6)) {
//			printf("It thinks it's done \n");
			m_robot->ResetElevatorEncoder();
			m_robot->isZeroingElevator = false;
		}
	} else if (m_robot->GetCarriageLimitSwitch() && !m_robot->isZeroingElevator
				&& m_robot->GetSecondStageLimitSwitch() && elevator_height<=InchesToMeters(6) && power < 0) {
	//	printf("Resetting the elevator encoder \n");
		power = 0;
		elevator_height = 0;
		m_robot->ResetElevatorEncoder();
	} else if (elevator_height >= max_up && !m_robot->isZeroingElevator && power > 0) {
		power = 0;
		elevator_height = max_up;
	}
	
	//printf("elevator speed fraction: %f\n",m_robot->elevatorSpeedFraction);
	m_robot->SetElevatorMotor(power);	
	m_robot->Unlock();
	m_last_elevator_height = elevator_height;
}
