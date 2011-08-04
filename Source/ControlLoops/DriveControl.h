#ifndef DRIVE_CONTROL_H
#define DRIVE_CONTROL_H

#include "RobotState.h"
#include "CSVReader.h"
#include "DiscreteAccelFilter.h"
#include "ContinuousAccelFilter.h"
#include "../../matlab/mat.cpp"

/**
 * Controls the drive of the robot
 */
class DriveControl
{
public:
	DriveControl(RobotState* robot, CSVReader* csvReader);
	~DriveControl();
	void Update();
	void Reset();
	
private:
	RobotState* m_robot;
	CSVReader* m_csvReader;
	
	// acceleration filters
	AccelFilterBase* m_straightFilter;
	AccelFilterBase* m_turnFilter;
	
	// current status info
	double m_currentA;
	double m_currentV;
	double m_currentX;
	double m_currentAngularA;
	double m_currentAngularV;
	double m_filtered_angle;
	double m_maxA;
	double m_old_wheel;
	double m_sumStoppedError;
	struct matrix *m_y;
	struct matrix *m_r;
	ss_controller m_ssc;
};

#endif // DRIVE_CONTROL_H
