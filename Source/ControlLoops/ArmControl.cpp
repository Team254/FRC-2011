#include "ArmControl.h"
#include "CommonIncludes.hpp"

ArmControl::ArmControl(RobotState* robot, CSVReader* csvReader)
{
	m_robot = robot;
	m_csvReader = csvReader;
	Reset();
}

void ArmControl::Reset()
{
	m_last_arm_angle=0.0;
}

void ArmControl::Update()
{
	m_robot->Lock();

	// control loop toggle
	if(!m_robot->controlLoopsOn)
	{
		m_robot->SetArmMotor(m_robot->armPower);
		m_robot->Unlock();
		return;
	}
	
	// set hard stops
	double max_up = DegreesToRadians(m_csvReader->GetValue("ARM_UP_ANGLE"));
	double max_down = DegreesToRadians(m_csvReader->GetValue("ARM_DOWN_ANGLE"));
	if (m_robot->armGoal <= max_down)
		m_robot->armGoal = max_down;
	if (m_robot->armGoal >= max_up)
		m_robot->armGoal = max_up;

	// This implements a control loop in order to put the arm at a goal.
	// The control loop must be in control of the motor at all times.
	// Any exceptions need to be run by Austin Schuh.

	// compute error and derivative
	static double integral=0.0;
	double arm_angle = m_robot->GetArmAngle();
	double angle_error = m_robot->armGoal - arm_angle;
	double angle_deriv = arm_angle - m_last_arm_angle;
	integral+=angle_error;
	
	if(integral>1)
		integral=1;
	if(integral<-1)
		integral=-1;
	
	m_robot->armVelocity = angle_deriv / m_robot->dt;

	double kp;
	double ki;
	double kd;
	// Gain schedule the constants.
	// The arm acts differently when it is driving up, vs driving down.
	// Use two different constants for that.
	if (angle_error > 0) {
		kp = m_csvReader->GetValue("KP_ARM_UP");
		ki = m_csvReader->GetValue("KI_ARM_UP");
		kd = m_csvReader->GetValue("KD_ARM_UP");
	} else {
		kp = m_csvReader->GetValue("KP_ARM_DOWN");
		ki = m_csvReader->GetValue("KI_ARM_DOWN");
		kd = m_csvReader->GetValue("KD_ARM_DOWN");
	}

	// Compute the power to send to the arm.
	double power = kp * angle_error + ki * integral + kd * angle_deriv;

	// Don't let the robot apply power in the direction of a limit when it is at the limit.
	if (arm_angle > DegreesToRadians(90.0) && power > 0) {
		power = 0;
	} else if (arm_angle < DegreesToRadians(-10.0) && power < 0) {
		power = 0;
	}

	//printf("arm stopped: %d\n",m_robot->armStopped());
	// Write the power out to the motor.
	m_robot->SetArmMotor(power);
	m_robot->Unlock();
	m_last_arm_angle = arm_angle;
}

