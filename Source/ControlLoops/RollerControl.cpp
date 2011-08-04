#include "RollerControl.h"
#include "CommonIncludes.hpp"

RollerControl::RollerControl(RobotState* robot, CSVReader* csvReader)
{
	m_robot = robot;
	m_csvReader = csvReader;
}

void RollerControl::Reset()
{
}

void RollerControl::Update()
{
	m_robot->Lock();
	m_robot->SetGrabberOpen(m_robot->grabberOpen);
	
	if(m_robot->grabberOpen) {
		m_robot->isRollingIn = false;
	}
	
	// control loop toggle
	if (!m_robot->controlLoopsOn) {
		if (m_robot->isRollingIn) {
			// Suck in
			m_robot->SetBottomRollerMotor(1.0);
			m_robot->SetTopRollerMotor(1.0);
		} else if (m_robot->isRollingOut) {
			// Spit out
			m_robot->SetBottomRollerMotor(-1.0);
			m_robot->SetTopRollerMotor(-1.0);
		} else {
			// auto roll slowly if the grabber is open
			if(m_robot->grabberOpen) {
				double spitPower;
				if(DriverStation::GetInstance()->IsAutonomous()) {
					spitPower=m_csvReader->GetValue("AUTO_ROLLER_SPIT_PWM");
				} else {
					spitPower=m_csvReader->GetValue("TELEOP_ROLLER_SPIT_PWM");
				}

				m_robot->SetBottomRollerMotor(-spitPower);
				m_robot->SetTopRollerMotor(-spitPower);
			} else {
				m_robot->SetBottomRollerMotor(0.0);
				m_robot->SetTopRollerMotor(0.0);
			}
		}
		m_robot->Unlock();
	}

	else if (m_robot->isRollingIn) {
		// If it's spinning,
		if (fabs(m_robot->rollerSpinVal) > .01) {
			if (m_robot->rollerSpinVal > .0) {
				// Rotate down
    			if (!m_robot->GetTubeLimitSwitch()) {
    				// Pull the tube in seriously if it's out,
    				// Let it spin if it wants.
					m_robot->SetBottomRollerMotor(1 + m_robot->rollerSpinVal / 2.0);
					m_robot->SetTopRollerMotor(1 - m_robot->rollerSpinVal / 2.0);
    			} else {
					// Got tube, let it slowly escape so it doesn't jam.
					m_robot->SetBottomRollerMotor(m_robot->rollerSpinVal * 0.66);
					m_robot->SetTopRollerMotor(-m_robot->rollerSpinVal);
    			}
			} else {
				// Rotate up
    			if (!m_robot->GetTubeLimitSwitch()) {
					// Let the tube escape slowly.
					m_robot->SetBottomRollerMotor(m_robot->rollerSpinVal);
					m_robot->SetTopRollerMotor(-m_robot->rollerSpinVal * 0.50);
				} else {
					// Pull it back in.
					m_robot->SetBottomRollerMotor(m_robot->rollerSpinVal + .2);
					m_robot->SetTopRollerMotor(-m_robot->rollerSpinVal * 0.50);
				}
			}
		} else {
			// No spinning, so just suck in if it's not pressed.
    		if (!m_robot->GetTubeLimitSwitch()) {
				m_robot->SetBottomRollerMotor(1.0);
				m_robot->SetTopRollerMotor(1.0);
   			} else {
				//stop if holding tube
				m_robot->SetBottomRollerMotor(0.0);
				m_robot->SetTopRollerMotor(0.0);
   			}
		}
	} else if (m_robot->isRollingOut) {
		// Eject tube.
		m_robot->SetBottomRollerMotor(-1.0);
		m_robot->SetTopRollerMotor(-1.0);
	} else {
		if (m_robot->grabberOpen) {
			// auto roll slowly if the grabber is open
			double spitPower;
			if(DriverStation::GetInstance()->IsAutonomous()) {
				spitPower=m_csvReader->GetValue("AUTO_ROLLER_SPIT_PWM");
			} else {
				spitPower=m_csvReader->GetValue("TELEOP_ROLLER_SPIT_PWM");
			}

			m_robot->SetBottomRollerMotor(-spitPower);
			m_robot->SetTopRollerMotor(-spitPower);
		} else {
			m_robot->SetBottomRollerMotor(0.0);
			m_robot->SetTopRollerMotor(0.0);
		}
	}
	m_robot->Unlock();
}

