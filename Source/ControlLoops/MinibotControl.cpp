#include "MinibotControl.h"
#include "CommonIncludes.hpp"

MinibotControl::MinibotControl(RobotState* robot, CSVReader* csvReader)
{
	m_robot = robot;
	m_csvReader = csvReader;
}

void MinibotControl::Reset()
{
}

void MinibotControl::Update()
{
	m_robot->Lock();
	if(!m_robot->isDisabled) {
		m_robot->m_lcd->PrintfLine(DriverStationLCD::kUser_Line2, "Time to Deploy: %06.3f", maximum(0.0,110 - m_robot->GetTime()));
		if (m_robot->minibotDeploy) {
			m_robot->m_lcd->PrintfLine(DriverStationLCD::kUser_Line3, "Minibot Armed");
		} else {
			m_robot->m_lcd->PrintfLine(DriverStationLCD::kUser_Line3, "Minibot Disabled");
		}
	}
	else
	{
		m_robot->m_lcd->PrintfLine(DriverStationLCD::kUser_Line2, "Robot Disabled");
		m_robot->m_lcd->PrintfLine(DriverStationLCD::kUser_Line3, "");
	}
	
	if(m_robot->isOperatorControl && !m_robot->isDisabled && m_robot->minibotRelease)
    	m_robot->SetMinibotRelease(true);
	else
    	m_robot->SetMinibotRelease(false);
	
	if(m_robot->isOperatorControl && m_robot->minibotDeploy &&
	   (!m_robot->controlLoopsOn || m_robot->GetTime()>=110.0))
		// if the switch is flipped and we're in operator control
		// and either the control loops are off or the timer is ready to roll
		m_robot->SetMinibotDeploy(true);
	else
		m_robot->SetMinibotDeploy(false);
	m_robot->Unlock();
}

