#include "DriveControl.h"
#include "CommonIncludes.hpp"
#include <fstream>
ofstream positionlogger("positionlog.log");
DriveControl::DriveControl(RobotState* robot, CSVReader* csvReader)
{
	m_robot = robot;
	m_csvReader = csvReader;
	
	// set up acceleration profile filtering classes
	m_straightFilter = new ContinuousAccelFilter();
	m_turnFilter = new ContinuousAccelFilter();

	m_y = init_matrix(2,1);
	m_r = init_matrix(4,1);
	
    m_y = init_matrix(2,1);
    m_r = init_matrix(4,1);
	Reset();

}

void DriveControl::Reset()
{
	delete m_straightFilter;
	delete m_turnFilter;
	m_straightFilter = new ContinuousAccelFilter();
	m_turnFilter = new ContinuousAccelFilter();
	m_currentA = 0.0;
	m_currentV = 0.0;
	m_currentX = 0.0;
	m_currentAngularA = 0.0;
	m_currentAngularV = 0.0;
	m_filtered_angle = 0.0;
	m_old_wheel = 0.0;
	//m_maxA = m_csvReader->GetValue("MAXA_DRIVE");
	//m_maxA = 2.0;
	m_robot->turnOffset = 0.0;
	m_sumStoppedError = 0.0;
	free_matrix(m_y);
	free_matrix(m_r);
	m_y = init_matrix(2,1);
	m_r = init_matrix(4,1);
	flash_matrix(m_y,0.0,0.0);
	flash_matrix(m_r,0.0,0.0,0.0,0.0);
	m_ssc.reset();
}

DriveControl::~DriveControl()
{
	delete m_straightFilter;
	delete m_turnFilter;
	free_matrix(m_y);
	free_matrix(m_r);
}

void DriveControl::Update()
{
	m_robot->Lock();
	m_robot->m_lcd->PrintfLine(DriverStationLCD::kUser_Line6, "gyro: %f deg",RadiansToDegrees(m_robot->GetGyroValue()));
	//assumption stuff
	m_robot->updateAssumption();
	//printf("left: %f right: %f gyro: %f\n",MetersToFeet(m_robot->GetLeftDistance()),MetersToFeet(m_robot->GetRightDistance()), RadiansToDegrees(m_robot->GetGyroValue()));
	//printf("assumed x: %f assumed y: %f assumed angle: %f\n",MetersToFeet(m_robot->assumed_xpos), MetersToFeet(m_robot->assumed_ypos), RadiansToDegrees(m_robot->assumed_theta));
	
	// allow to override if the joysticks are moved
	// NOTE: the control board is not updated during autonomous
	// so this will not override the control loops during autonomous
	if(m_robot->isControlLoopDriving && (m_robot->turnPower!=0 || m_robot->straightDrivePower!=0)) {
		printf("JOYSTICKS HIT, DRIVE CONTROLS CANCELLING\n");
		m_robot->straightDistanceGoal=0;
		m_robot->isControlLoopDriving=false;
	}
	if(m_robot->resetDriveControl) {
		Reset();
		m_robot->resetDriveControl = false;
	}
	if(m_robot->isControlLoopDriving)
	{	
    	m_straightFilter->CalcSystem(m_robot->straightDistanceGoal - m_currentX, m_currentV, m_robot->straightDistanceGoalVelocity, m_robot->straightDistanceMaxAcceleration, m_robot->straightDistanceMaxVelocity, m_robot->dt);
    	m_currentA=m_straightFilter->GetCurrAcc();
    	m_currentV=m_straightFilter->GetCurrVel();
    	m_currentX=m_straightFilter->GetCurrPos();
    	
    	m_turnFilter->CalcSystem(m_robot->turnAngleGoal - m_filtered_angle, m_currentAngularV, m_robot->turnAngleGoalVelocity, 8 , M_PI * 2.0 / 3.0, m_robot->dt);
    	m_currentAngularA = m_turnFilter->GetCurrAcc();
    	m_currentAngularV = m_turnFilter->GetCurrVel();
    	m_filtered_angle  = m_turnFilter->GetCurrPos();
    
    	static const double robotWidth = 0.61799;
    	double theta_gyro = m_robot->GetGyroValue();
    	double theta_measured = (m_robot->GetRightDistance()-m_robot->GetLeftDistance())/robotWidth;
    	double kI=0.017;
    	//double kI=0.015;
    	// If we are at the goal (theoretically...), add in an integral.
    	// Kill this integral as soon as we try to move to not change the real goal.
		// When the robot turns, the extra offset won't really help.
    	// But kill it gradually to not upset the controller and cause it to pulse the angle.
		
		// Enable the I term when we are close.
    	if (fabs(m_robot->turnAngleGoal - m_filtered_angle) < 0.0001 
    				&&  fabs(RadiansToDegrees((m_robot->turnAngleGoal + m_robot->turnOffset) - theta_measured)) < 18.0) {
    		double KiTurn;
    		// If the arm is down, it's harder to turn...  Not sure why.
			// This might be componded by something else in the turn to grab the tube that was causing it to be slow.
    		//if (m_robot->armGoal < 0.0) {
    			//KiTurn = 0.0254;
    			//KiTurn = 0.040;
    		//} else {
    			//KiTurn = 0.0230;
    			KiTurn = 0.0254;
    			//KiTurn = 0.0200;
    		//}
    		m_sumStoppedError += ((m_robot->turnAngleGoal + m_robot->turnOffset) - theta_measured) * KiTurn;
    	} else {
    		// Derate the integral if we are turning so it doesn't take effect any more.
    		// Gradually so it doesn't cause the bot to rapidly turn.
    		if (!(fabs(m_robot->turnAngleGoal - m_filtered_angle) < 0.0001)) {
    			m_sumStoppedError *= 0.97;
    		}
    	}
    	// Limit the change in the offset to 0.01 rad / 100 of a second.
		// This will prevent the -big gyro bug from showing up.
		// I'm seeing a very large gyro value occasionally which is messing up
		// the offset and not letting it recover for a while.  Not good.
    	double doffset = ((theta_measured-m_robot->turnOffset)-theta_gyro)*kI;
    	if (doffset > 0.01) {
    		doffset = 0.01;
    	} else if (doffset < -0.01) {
    		doffset = -0.01;
    	}
    	m_robot->turnOffset += doffset;
    	
		
    	static int i=0;
    	static bool printing=false;
    	if(printing && i%1==0) {
    		printf("left: %f right: %f gyro: %f\n",m_robot->GetLeftDistance(),m_robot->GetRightDistance(),m_robot->GetGyroValue());
    		printf("offset: %f stoppederror: %f\n",m_robot->turnOffset,m_sumStoppedError);
        	printf("error: %f ",(theta_measured-m_robot->turnOffset)-theta_gyro);
        	//printf("offset: %f ",m_robot->turnOffset);
        	//printf("ioffset: %f ",m_sumStoppedError);
        	printf("measured: %f ",theta_measured);
        	printf("gyro: %f ",theta_gyro);
        	//printf("drive err %f ",m_robot->straightDistanceGoal - (m_robot->GetLeftDistance()+m_robot->GetRightDistance())/2);
        	printf("angle err %f ",(m_robot->turnAngleGoal + m_robot->turnOffset) - theta_measured);
        	//printf("angle goal: %f ",RadiansToDegrees(m_robot->turnAngleGoal));
        	printf("\n");
    	}
    	i++;
    	
    	double angleFactor = m_filtered_angle*robotWidth/2;
    	double angularVelocityFactor = m_currentAngularV*robotWidth/2;
    	
	//log stuff
    	if(m_robot->isAutonomous && !m_robot->isDisabled) {
    		positionlogger << m_robot->GetTime() << ", "; //1
    		positionlogger << m_robot->assumed_xpos << ", "; //2 
    		positionlogger << m_robot->assumed_ypos << ", "; //3 
    		positionlogger << theta_gyro << ", "; //4 
    		positionlogger << m_robot->turnOffset << ", "; //5
    		positionlogger << m_robot->GetLeftDistance() << ", "; //6
    		positionlogger << m_robot->GetRightDistance() << ", "; //7
    		positionlogger << (theta_measured-m_robot->turnOffset)-theta_gyro << ", "; //8
    		positionlogger << (theta_measured-m_robot->turnOffset) << ", "; //9
    		positionlogger << m_currentX-(m_robot->turnOffset + m_sumStoppedError)*robotWidth/2.0-angleFactor << ", "; //10
    		positionlogger << m_currentX+(m_robot->turnOffset + m_sumStoppedError)*robotWidth/2.0+angleFactor << ", "; //11
    		positionlogger << m_sumStoppedError << endl; //12
    	}
    
	//setup the output matrix
    	struct matrix* outputs;
    	outputs = init_matrix(num_outputs, 1);
    	flash_matrix(m_r,m_currentX-(m_robot->turnOffset + m_sumStoppedError)/2.0-angleFactor,m_currentV-angularVelocityFactor,m_currentX+(m_robot->turnOffset + m_sumStoppedError)/2.0+angleFactor,m_currentV+angularVelocityFactor);
    	flash_matrix(m_y,m_robot->GetLeftDistance(),m_robot->GetRightDistance());
    	m_ssc.update(outputs,m_r,m_y);
    	if (m_robot->ignoreTurnControlLoop) {
    		outputs->data[0] = outputs->data[1] = (outputs->data[0] + outputs->data[1]) / 2.0;
    	}
    	
    	if (maximum(fabs(outputs->data[0]), fabs(outputs->data[1])) > 12.0) {
    		//printf("scaling\n");
    		//double turnPower = outputs->data[0] - outputs->data[1];
    		//double drivePower = outputs->data[0] + outputs->data[1];
    		double scaleFactor = 12.0 / maximum(fabs(outputs->data[0]), fabs(outputs->data[1]));
	    	
    		/*
    		if (fabs(turnPower) < 0.5 * fabs(drivePower)) {
    			double deltaTurn = turnPower / 2.0 / scaleFactor * 0.4;
    			outputs->data[0] += deltaTurn;
    			outputs->data[1] -= deltaTurn;
    			scaleFactor = 12.0 / maximum(fabs(outputs->data[0]), fabs(outputs->data[1]));
    		} else  if (0.5 * fabs(turnPower) > fabs(drivePower)) {
    			double deltaDrive = turnPower / 2.0 / scaleFactor * 0.4;
    			outputs->data[0] += deltaDrive;
    			outputs->data[1] += deltaDrive;
    			scaleFactor = 12.0 / maximum(fabs(outputs->data[0]), fabs(outputs->data[1]));
    		}*/
  			outputs->data[0] *= scaleFactor;
    		outputs->data[1] *= scaleFactor;
    	}
    	
    	double outl=outputs->data[0]/12;
    	double outr=outputs->data[1]/12;
    	if(fabs(outl)>1.0)
    		outl/=fabs(outl);
    	if(fabs(outr)>1.0)
    		outr/=fabs(outr);
    	if(printing && i%1==0) {
    		printf("left pwm: %f right pwm: %f\n",outl,outr);
    	}
    	m_robot->SetLeftMotor_Linearized(outl);
    	m_robot->SetRightMotor_Linearized(outr);
    	free_matrix(outputs);
	} else {
		double throttle = -m_robot->straightDrivePower;
		double wheel = m_robot->turnPower;
		bool isQuickTurn = m_robot->isQuickTurn;
		bool isHighGear = m_robot->isHighGear;
		//printf("Drive Distance ld %f rd %f\n", m_robot->GetLeftDistance(), m_robot->GetRightDistance());
		
		double wheelNonLinearity;
		
		double neg_inertia = wheel - m_old_wheel;
		m_old_wheel = wheel;
		
		//triple sine wave ftw!
		if (isHighGear) {
			wheelNonLinearity = m_csvReader->GetValueWithDefault("TURN_NONLIN_HIGH", 0.1);
			// Apply a sin function that's scaled to make it feel better.
			wheel = sin(M_PI / 2.0 * wheelNonLinearity * wheel) / sin(M_PI / 2.0 * wheelNonLinearity);
			wheel = sin(M_PI / 2.0 * wheelNonLinearity * wheel) / sin(M_PI / 2.0 * wheelNonLinearity);
		} else {
			wheelNonLinearity = m_csvReader->GetValueWithDefault("TURN_NONLIN_LOW", 0.1);
			// Apply a sin function that's scaled to make it feel better.
			wheel = sin(M_PI / 2.0 * wheelNonLinearity * wheel) / sin(M_PI / 2.0 * wheelNonLinearity);
			wheel = sin(M_PI / 2.0 * wheelNonLinearity * wheel) / sin(M_PI / 2.0 * wheelNonLinearity);
			wheel = sin(M_PI / 2.0 * wheelNonLinearity * wheel) / sin(M_PI / 2.0 * wheelNonLinearity);
		}
		
		double left_pwm, right_pwm, overPower;
		float sensitivity = 1.7;
		
		float angular_power;
		float linear_power;
		
		//negative inertia!
		static double neg_inertia_accumulator = 0.0;
		double neg_inertia_scalar;
		if (isHighGear) {
			neg_inertia_scalar = m_csvReader->GetValueWithDefault("NEG_INERTIA_HIGH", 0.0);
			sensitivity = m_csvReader->GetValueWithDefault("SENSE_HIGH", 1.7);
		} else {
			if (wheel * neg_inertia > 0) {
				neg_inertia_scalar = m_csvReader->GetValueWithDefault("NEG_INERTIA_LOW_MORE", 0.0);
			} else {
				if (fabs(wheel) > 0.65) {
					neg_inertia_scalar = m_csvReader->GetValueWithDefault("NEG_INERTIA_LOW_LESS_EXT", 0.0);
				} else {
					neg_inertia_scalar = m_csvReader->GetValueWithDefault("NEG_INERTIA_LOW_LESS", 0.0);
				}
			}
			sensitivity = m_csvReader->GetValueWithDefault("SENSE_LOW", 1.2);
			
			if (fabs(throttle) > m_csvReader->GetValueWithDefault("SENSE_CUTTOFF", 0.1)) {
				sensitivity = 1 - (1 - sensitivity) / fabs(throttle);
			}
		}
		double neg_inertia_power=neg_inertia * neg_inertia_scalar;
		neg_inertia_accumulator+=neg_inertia_power;
		
		
		
		wheel = wheel + neg_inertia_accumulator;
		if(neg_inertia_accumulator>1)
			neg_inertia_accumulator-=1;
		else if (neg_inertia_accumulator<-1)
			neg_inertia_accumulator+=1;
		else
			neg_inertia_accumulator=0;
		
		linear_power = throttle;
		
		//quickturn!
		if (isQuickTurn) {
			overPower = 1.0;
			if (isHighGear) {
				sensitivity = 1.0;
			} else {
				sensitivity = 1.0;
			}
			angular_power = wheel;
		} else {
			overPower = 0.0;
			angular_power = fabs(throttle) * wheel * sensitivity;
		}
		
		right_pwm = left_pwm = linear_power;
		left_pwm += angular_power;
		right_pwm -= angular_power;
		
		if (left_pwm > 1.0) {
			right_pwm -= overPower*(left_pwm - 1.0);
			left_pwm = 1.0;
		} else if (right_pwm > 1.0) {
			left_pwm -= overPower*(right_pwm - 1.0);
			right_pwm = 1.0;
		} else if (left_pwm < -1.0) {
			right_pwm += overPower*(-1.0 - left_pwm);
			left_pwm = -1.0;
		} else if (right_pwm < -1.0) {
			left_pwm += overPower*(-1.0 - right_pwm);
			right_pwm = -1.0;
		}
		//printf("left pwm: %f right pwm: %f\n",left_pwm,right_pwm);
		//printf("left wheel: %f right wheel: %f\n",m_robot->GetLeftDistance(),m_robot->GetRightDistance());
		m_robot->SetLeftMotor_Linearized(left_pwm);
		m_robot->SetRightMotor_Linearized(right_pwm);
		m_robot->SetHighGear(isHighGear);
		
	}
	m_robot->Unlock();
}

