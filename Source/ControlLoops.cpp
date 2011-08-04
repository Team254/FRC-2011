#include "ControlLoops.h"
#include "RobotState.h"
#include "CSVReader.h"
#include "ControlLoops/ControllerList.h"
#include "WPILib.h"
#include "Notifier.h"

void* CallNotify(void* instance)
{
	((ControlLoops* )instance)->UpdateControlLoops();
	return NULL;
}

ControlLoops::ControlLoops(RobotState* robot, CSVReader* csvReader)
: m_robot(robot)
, m_csvReader(csvReader)
{
	shouldReset = false;
	m_driveControl = new DriveControl(robot, csvReader);
	m_rollerControl = new RollerControl(robot, csvReader);
	m_armControl = new ArmControl(robot, csvReader);
	m_elevatorControl = new ElevatorControl(robot, csvReader);
	m_minibotControl = new MinibotControl(robot, csvReader);
	//run the control loops on a notifier thread
	m_notifier = new Notifier((TimerEventHandler)CallNotify, (void*)this);
	m_notifier->StartPeriodic(m_robot->dt);
}

void ControlLoops::Reset()
{
	m_robot->Lock();
	shouldReset = true;
	m_robot->Unlock();
}

void ControlLoops::UpdateControlLoops()
{
	m_robot->Lock();
	m_robot->m_lcd->UpdateLCD();        
	static bool printVals=true;
	if(printVals) {
		/*
		printf("LEFT ENCODER: %f\n",m_robot->GetLeftDistance()/(2.0 * M_PI * InchesToMeters(3.5) / 2.0));
		printf("RIGHT ENCODER: %f\n",m_robot->GetRightDistance()/(2.0 * M_PI * InchesToMeters(3.5) / 2.0));
		printf("GYRO: %f\n",RadiansToDegrees(m_robot->GetGyroValue()));
		printf("\n");
		*/
	}
	if (shouldReset) {
		shouldReset = false;
		m_robot->Unlock();
		m_driveControl->Reset();
		m_rollerControl->Reset();
		m_armControl->Reset();
		m_elevatorControl->Reset();
		m_minibotControl->Reset();
	} else {
		m_robot->Unlock();
	}
	m_driveControl->Update();
	m_rollerControl->Update();
	m_armControl->Update();
	if(m_robot->elevatorEnabled)
		m_elevatorControl->Update();
	m_minibotControl->Update();
	//go ahead and update the robot's lcd here
	//because it's updated universally every 10ms
}

ControlLoops::~ControlLoops()
{
	m_notifier->Stop();
	delete m_notifier;
	delete m_driveControl;
	delete m_rollerControl;
	delete m_armControl;
	delete m_elevatorControl;
	delete m_minibotControl;
}
