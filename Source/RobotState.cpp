#include "RobotState.h"
#include "RobotPorts.h"
#include "CSVReader.h"
#include "CommonIncludes.hpp"

RobotState::RobotState(CSVReader* csvReader)
{
	
	m_csvReader=csvReader;
	printf("Trying high %f\n", m_csvReader->GetValueWithDefault("SENSE_HIGH", 1.7));
	
	//create semaphore
	m_lock=semMCreate(SEM_Q_PRIORITY |
					 SEM_DELETE_SAFE |
					 SEM_INVERSION_SAFE);
	
	// drivetrain
	m_leftMotor1  = new Victor(LEFT_MOTOR_A_PWM_PORT);
	m_leftMotor2  = new Victor(LEFT_MOTOR_B_PWM_PORT);
	m_rightMotor1 = new Victor(RIGHT_MOTOR_A_PWM_PORT);
	m_rightMotor2 = new Victor(RIGHT_MOTOR_B_PWM_PORT);
	m_leftEncoder = new Encoder(LEFT_WHEEL_ENCODER_A_PWM_PORT, LEFT_WHEEL_ENCODER_B_PWM_PORT, true);
	m_rightEncoder= new Encoder(RIGHT_WHEEL_ENCODER_A_PWM_PORT, RIGHT_WHEEL_ENCODER_B_PWM_PORT, true);
	EnableLeftEncoder();
	EnableRightEncoder();
	m_gearShiftSolenoid = new Solenoid(GEAR_SHIFT_SOLENOID_CHAN);
	
	// elevator
	m_elevatorMotor1 = new Victor(SPOOL_MOTOR_A_PWM_PORT);
	m_elevatorMotor2 = new Victor(SPOOL_MOTOR_B_PWM_PORT);
	m_elevatorEncoder = new Encoder(ELEVATOR_ENCODER_A_PWM_PORT, ELEVATOR_ENCODER_B_PWM_PORT, true);
	EnableElevatorEncoder();
	m_secondStageSwitch = new DigitalInput(ELEVATOR_SECOND_STAGE_LIMIT_SWITCH);
	// arm
	m_armMotor = new Victor(ARM_MOTOR_PWM_PORT);
	m_armPot = new AnalogChannel(ARM_POT_PORT);
	m_grabberSolenoid = new DoubleSolenoid(ROLLER_HIGH_SOLENOID_CHAN, ROLLER_LOW_SOLENOID_CHAN);
	m_tubeSwitch = new DigitalInput(ROLLER_LIMIT_SWITCH);
	m_carriageSwitch = new DigitalInput(CARRIAGE_LIMIT_SWITCH);

	// rollers
	m_topRollerMotor = new Victor(TOP_ROLLER_MOTOR_PWM_PORT);
	m_bottomRollerMotor = new Victor(BOTTOM_ROLLER_MOTOR_PWM_PORT);
	
	// other
	m_compressor = new Compressor(COMPRESSOR_PRESSURE_SWITCH_CHAN, COMPRESSOR_RELAY_CHAN);
	EnableCompressor();
	m_minibotReleaseSolenoid = new Solenoid(MINIBOT_RELEASE_SOLENOID);
	m_minibotDeploySolenoid = new Solenoid(MINIBOT_DEPLOY_SOLENOID);
	m_gyro = new Gyro(GYRO_PORT);
	//2011 gyro:
	m_gyro->SetSensitivity(0.0005);
	m_gyro->Reset();
	m_timer = new Timer();
	m_timer->Start();
	
	m_lcd = DriverStationLCD::GetInstance();
	autonomousDescription = "";
	
	isDisabled = true;
	
	ResetState();
}

RobotState::~RobotState() {
	//release semaphore
	DisableCompressor();
	semTake(m_lock,WAIT_FOREVER);
	semDelete(m_lock);
	delete m_gyro;
	delete m_minibotDeploySolenoid;
	delete m_minibotReleaseSolenoid;
	delete m_compressor;
	delete m_timer;
	delete m_bottomRollerMotor;
	delete m_topRollerMotor;
	delete m_grabberSolenoid;
	delete m_armPot;
	delete m_armMotor;
	delete m_carriageSwitch;
	delete m_secondStageSwitch;
	delete m_tubeSwitch;
	delete m_elevatorEncoder;
	delete m_elevatorMotor2;
	delete m_elevatorMotor1;
	delete m_gearShiftSolenoid;
	delete m_rightEncoder;
	delete m_leftEncoder;
	delete m_rightMotor2;
	delete m_rightMotor1;
	delete m_leftMotor2;
	delete m_leftMotor1;
}

// Sensor getters
double RobotState::GetLeftDistance() const
{
	return m_leftEncoder->Get() / 256.0 * 2.0 * M_PI * InchesToMeters(3.5) / 2.0;
}

double RobotState::GetRightDistance() const
{
	// compensate for right encoder being mounted backwards
	return -m_rightEncoder->Get() / 256.0 * 2.0 * M_PI * InchesToMeters(3.5) / 2.0;
}

double RobotState::GetGyroValue() const
{
	return DegreesToRadians(m_gyro->GetAngle());
}

double RobotState::GetArmAngle() const
{
	static const double lowArmAngle = -10.0;
	static const double highArmAngle = 90.0;
	static const double angleDiff = fabs(highArmAngle - lowArmAngle);
	static const double angleBase = lowArmAngle;

	const double lowArmVoltage = m_ArmLowVoltage;
	const double highArmVoltage = m_ArmHighVoltage;
	const double voltageDiff = fabs(highArmVoltage - lowArmVoltage);
	const double voltageBase = lowArmVoltage;

	double armVoltage = m_armPot->GetVoltage();
	if (armVoltage < 0.0)
		armVoltage = 0.0;
	
	double degrees = (((angleDiff)/(voltageDiff)) * (armVoltage - voltageBase)) + angleBase;
	printf("voltage: %f angle: %f\n",armVoltage, degrees);
	// This is debug code for testing arm pot values at different angles
	if (m_csvReader->GetValue("DEBUG_ARM_POT_VOLTAGE") != 0.0) {
		static int count=0;
		if (count++ % 100 == 0) {
			printf("Voltage:%f\tDegrees:%f \n", armVoltage, degrees);
		}
	}

	return (degrees * M_PI) / 180.0;	
}

double RobotState::GetElevatorHeightValue() const
{
	double gearingRatio=(19.0 / 45.0);
	double spoolRadius=InchesToMeters(1);
	double encoderVal = m_elevatorEncoder->Get() / 256.0;
	return encoderVal * gearingRatio * spoolRadius * 2 * M_PI;	
}

bool RobotState::GetTubeLimitSwitch() const
{
	return (bool) m_tubeSwitch->Get();
}

bool RobotState::hasTube()
{
	Lock();
	bool ans = (bool) m_tubeSwitch->Get();
	Unlock();
	return ans;
}

void RobotState::waitForTube()
{
	while (true) {
		if (hasTube()) {
			return;
		}
		Wait(dt);
	}
}

void RobotState::closeClaw()
{
	Lock();
	grabberOpen = false;
	Unlock();
}
void RobotState::grabTube()
{
	Lock();
	isRollingIn = true;
	isRollingOut = false;
	Unlock();
}

void RobotState::openClaw()
{
	Lock();
	grabberOpen = true;
	Unlock();
}

void RobotState::ejectTube()
{
	Lock();
	isRollingIn = false;
	isRollingOut = true;
	Unlock();
}

void RobotState::ignoreTube()
{
	Lock();
	isRollingIn = false;
	isRollingOut = false;
	Unlock();
}

bool RobotState::GetCarriageLimitSwitch() const
{
	return (bool) m_carriageSwitch->Get();
}

bool RobotState::GetSecondStageLimitSwitch() const

{
	return !(bool) m_secondStageSwitch->Get();
}


// Output setters
void RobotState::SetLeftMotor(double pwm)
{
	if (pwm > 1.0) pwm = 1.0;
	if (pwm < -1.0) pwm = -1.0;
	m_leftMotor1->Set(pwm);
	m_leftMotor2->Set(pwm);
}

void RobotState::SetLeftMotor_Linearized(double pwm)
{
	SetLeftMotor(victor_linearize(pwm));
}

void RobotState::SetRightMotor(double pwm)
{
	if (pwm > 1.0) pwm = 1.0;
	if (pwm < -1.0) pwm = -1.0;
	m_rightMotor1->Set(-pwm);
	m_rightMotor2->Set(-pwm);
}

void RobotState::turnAngleDegrees(double angle)
{
	Lock();
	isControlLoopDriving=true;
	turnAngleGoal += DegreesToRadians(angle);
	turnAngleGoalVelocity = 0.0;
	Unlock();
}

void RobotState::enableSteering(bool isEnabled)
{
	Lock();
	ignoreTurnControlLoop = !isEnabled;
	Unlock();
}

void RobotState::driveForwardsFeetAtVelocity(double feet, double velocity)
{
	Lock();
	isControlLoopDriving=true;
	straightDistanceGoal += FeetToMeters(feet);
	straightDistanceGoalVelocity = 0.0;
	straightDistanceMaxVelocity = FeetToMeters(velocity);
	Unlock();
}

void RobotState::driveForwardsFeet(double feet)
{
	Lock();
	isControlLoopDriving=true;
	straightDistanceGoal += FeetToMeters(feet);
	straightDistanceGoalVelocity = 0.0;
	straightDistanceMaxVelocity = m_csvReader->GetValue("MAXV_DRIVE");
	Unlock();
}

void RobotState::SetRightMotor_Linearized(double pwm)
{
	SetRightMotor(victor_linearize(pwm));
}

void RobotState::SetElevatorMotor(double pwm)
{
	if (pwm > 1.0) pwm = 1.0;
	if (pwm < -1.0) pwm = -1.0;
	m_elevatorMotor1->Set(-pwm);
	m_elevatorMotor2->Set(-pwm);
}

void RobotState::SetElevatorMotor_Linearized(double pwm)
{
	SetElevatorMotor(victor_linearize(pwm));
}

void RobotState::SetArmMotor(double pwm)
{
	if (pwm > 1.0) pwm = 1.0;
	if (pwm < -1.0) pwm = -1.0;
	m_armMotor->Set(-pwm);
}

void RobotState::SetArmMotor_Linearized(double pwm)
{
	//set up linearization
	SetArmMotor(victor_linearize(pwm));
}

void RobotState::SetTopRollerMotor(double pwm)
{
	if (pwm > 1.0) pwm = 1.0;
	if (pwm < -1.0) pwm = -1.0;
	m_topRollerMotor->Set(pwm);
}

void RobotState::SetTopRollerMotor_Linearized(double pwm)
{
	SetTopRollerMotor(victor_linearize(pwm));
}

void RobotState::SetBottomRollerMotor(double pwm)
{
	if (pwm > 1.0) pwm = 1.0;
	if (pwm < -1.0) pwm = -1.0;
	m_bottomRollerMotor->Set(pwm);
}

void RobotState::SetBottomRollerMotor_Linearized(double pwm)
{
	SetBottomRollerMotor(victor_linearize(pwm));
}

void RobotState::SetMinibotRelease(bool on)
{
	m_minibotReleaseSolenoid->Set(on);
}

void RobotState::SetMinibotDeploy(bool on)
{
	m_minibotDeploySolenoid->Set(on);
}

void RobotState::SetHighGear(bool high)
{
	m_gearShiftSolenoid->Set(high==false);
}

void RobotState::SetGrabberOpen(bool open)
{
	if (open) {
		m_grabberSolenoid->Set(DoubleSolenoid::kReverse);
	} else {
		m_grabberSolenoid->Set(DoubleSolenoid::kForward);
	}
}

void RobotState::elevatorHeight(double height)
{
	Lock();
	elevatorGoal = height;
	if (elevatorGoal < 0.01) {
		isZeroingElevator = true;
	}
	Unlock();
}

void RobotState::armAngleDegrees(double angle)
{
	Lock();
	armGoal = DegreesToRadians(angle);
	Unlock();
}

void RobotState::waitForArm()
{
	while (true) {
		Lock();
		if (armStopped()) {
			Unlock();
			return;
		}
		Unlock();
		Wait(dt);
	}
}

void RobotState::waitForArmWithTimeout(double timeout)
{
	while (true) {
		Lock();
		//printf("ARM STOPPED: %d CURRENT TIME: %f\n",armStopped(),GetTime());
		if (armStopped() || GetTime()>=timeout) {
			Unlock();
			return;
		}
		Unlock();
		Wait(dt);
	}
}


bool RobotState::armStopped()
{
	double angle_error = armGoal - GetArmAngle();
	return fabs(angle_error) < DegreesToRadians(8);
}
bool RobotState::driveStoppedWithinDegrees(double deg)
{
	double currPos = (GetLeftDistance() + GetRightDistance()) / 2.0;
	if (ignoreTurnControlLoop) {
		return fabs(straightDistanceGoal - currPos) <= InchesToMeters(1);
	} else {
		return fabs(straightDistanceGoal - currPos) <= InchesToMeters(1) && fabs(turnAngleGoal - GetGyroValue()) < DegreesToRadians(deg);
	}
}
bool RobotState::driveStopped()
{
	return driveStoppedWithinDegrees(5.0);
}
bool RobotState::driveStoppedFine()
{
	return driveStoppedWithinDegrees(1.5);
}

void RobotState::resetDriveGoal()
{
	straightDistanceGoal = 0;
}

void RobotState::waitForDriveWithinDegrees(double deg)
{
	while (true) {
		Lock();
		if (driveStoppedWithinDegrees(deg)) {
			Unlock();
			return;
		}
		Unlock();
		Wait(dt);
	}
}
void RobotState::waitForDriveFine()
{
	while (true) {
		Lock();
		if (driveStoppedFine()) {
			Unlock();
			return;
		}
		Unlock();
		Wait(dt);
	}
}
void RobotState::waitForDrive()
{
	while (true) {
		Lock();
		if (driveStopped()) {
			Unlock();
			return;
		}
		Unlock();
		Wait(dt);
	}
}
void RobotState::waitForDriveFeetLeft(double feet)
{
	while (true) {
		Lock();
		double currPos = (GetLeftDistance() + GetRightDistance()) / 2.0;
		if (fabs(straightDistanceGoal - currPos) <= FeetToMeters(feet)) {
			Unlock();
			return;
		}
		Unlock();
		Wait(dt);
	}
}

bool RobotState::elevatorStopped()
{
	double height_error = elevatorGoal - GetElevatorHeightValue();
	return fabs(height_error) < InchesToMeters(4.0);
}

void RobotState::waitForElevator()
{
	while (true) {
		Lock();
		if (elevatorStopped()) {
			Unlock();
			return;
		}
		Unlock();
		Wait(dt);
	}
}

//Resetters
void RobotState::ResetState()
{
	ResetGyro();
	ResetEncoders();
	ResetTimer();
	resetDriveControl = false;
	turnOffset = 0.0;
	straightDrivePower = 0.0;
	turnPower = 0.0;
	isQuickTurn = false;
	isHighGear = true;
	isControlLoopDriving = false;
	straightDistanceGoal = 0.0;
	straightDistanceGoalVelocity = 0.0;
	straightDistanceMaxVelocity = m_csvReader->GetValue("MAXV_DRIVE");
	straightDistanceMaxAcceleration = 2.0;
	ignoreTurnControlLoop = false;
	turnAngleGoal = 0.0;
	turnAngleGoalVelocity = 0.0;
	elevatorPower = 0.0;
	elevatorEnabled = true;
	isZeroingElevator = true;
	elevatorGoal = 0.0;
	elevatorVelocity = 0.0;
	isGrounding = false;
	isSetpointing = false;
	armPower = 0.0;
	armVelocity = 0.0;
	armGoal = DegreesToRadians(m_csvReader->GetValue("ARM_UP_ANGLE"));
	m_ArmHighVoltage=m_csvReader->GetValue("ARM_HIGH_VOLTAGE");
	m_ArmLowVoltage=m_csvReader->GetValue("ARM_LOW_VOLTAGE");
	currAutonomous = RobotState::kTwoTubeRight;
	autonomousDescription = "Auto: Two Tube Right";
	rollerSpinVal = 0.0;
	grabberOpen = false;
	isRollingIn = false;
	isRollingOut = false;
	minibotRelease = false;
	minibotDeploy = false;
	controlLoopsOn = true;	
	isOperatorControl = false;
	isAutonomous = false;
	prev_left = 0.0;
	prev_right = 0.0;
	prev_angle = 0.0;
	assumed_xpos = 0.0;
	assumed_ypos = 0.0;
	assumed_theta = 0.0;
	assumedTurnOffset = 0.0;
	SetBottomRollerMotor(0.0);
	SetTopRollerMotor(0.0);
	SetLeftMotor(0.0);
	SetRightMotor(0.0);
	SetElevatorMotor(0.0);
	SetArmMotor(0.0);
}

void RobotState::updateAssumption()
{
	static const double robotWidth = 0.61799;
	static const double kI = 0.017;
	
	const double leftDistance = GetLeftDistance();
	const double rightDistance = GetRightDistance();
	//calculating the offset using code from the drive control loop
	double theta_gyro = GetGyroValue();
	double theta_measured = (rightDistance-leftDistance)/robotWidth;
	double doffset = ((theta_measured-assumedTurnOffset)-theta_gyro)*kI;
	if (doffset > 0.01) {
		doffset = 0.01;
	} else if (doffset < -0.01) {
		doffset = -0.01;
	}
	assumedTurnOffset += doffset;
	
	const double leftDistance_offset=leftDistance+assumedTurnOffset*robotWidth/2;
	const double rightDistance_offset=rightDistance-assumedTurnOffset*robotWidth/2;
	//delta left, right, angle, distance
	double deltal, deltar, deltaa, deltad;
	deltal=leftDistance_offset-prev_left;
	deltar=rightDistance_offset-prev_right;
	deltaa=(deltar-deltal)/robotWidth;
	deltad=(deltal+deltar)/2;
	prev_left=leftDistance_offset;
	prev_right=rightDistance_offset;
	
	assumed_theta+=deltaa;
	assumed_xpos+=deltad*cos(assumed_theta);
	assumed_ypos+=deltad*sin(assumed_theta);
}
double RobotState::GetAssumedXPos()
{
	double out;
	Lock();
	out=assumed_xpos;
	Unlock();
	return out;
}

double RobotState::GetAssumedYPos()
{
	double out;
	Lock();
	out=assumed_ypos;
	Unlock();
	return out;
}

double RobotState::GetAssumedTheta()
{
	double out;
	Lock();
	out=assumed_theta;
	Unlock();
	return out;
}

void RobotState::ResetEncoders()
{
	ResetLeftEncoder();
	ResetRightEncoder();
	ResetElevatorEncoder();
}

void RobotState::ResetGyro()
{
	m_gyro->Reset();
}


//Others
void RobotState::EnableLeftEncoder()
{
	m_leftEncoder->Start();
}

void RobotState::DisableLeftEncoder()
{
	m_leftEncoder->Stop();
}

void RobotState::ResetLeftEncoder()
{
	m_leftEncoder->Reset();
}


void RobotState::EnableRightEncoder()
{
	m_rightEncoder->Start();
}

void RobotState::DisableRightEncoder()
{
	m_rightEncoder->Stop();
}

void RobotState::ResetRightEncoder()
{
	m_rightEncoder->Reset();
}


void RobotState::EnableElevatorEncoder()
{
	m_elevatorEncoder->Start();
}

void RobotState::DisableElevatorEncoder()
{
	m_elevatorEncoder->Stop();
}

void RobotState::ResetElevatorEncoder()
{
	m_elevatorEncoder->Reset();
}


void RobotState::EnableCompressor()
{
	m_compressor->Start();
}

void RobotState::DisableCompressor()
{
	m_compressor->Stop();
}

void RobotState::shiftHighGear()
{
	Lock();
	isHighGear=true;
	//SetHighGear(isHighGear);
	Unlock();
}
void RobotState::shiftLowGear()
{
	Lock();
	isHighGear=false;
	//SetHighGear(isHighGear);
	Unlock();
}

void RobotState::ResetTimer()
{
	m_timer->Reset();
}

double RobotState::GetTime()
{
	return m_timer->Get();
}

double RobotState::threadsafeTime()
{
	double out;
	Lock();
	out=GetTime();
	Unlock();
	return out;
}

double RobotState::victor_linearize(double goal_speed)
{
	const double deadband_value = 0.082;
	if (goal_speed > deadband_value)
		goal_speed -= deadband_value;
	else if (goal_speed < -deadband_value)
		goal_speed += deadband_value;
	else
		goal_speed = 0.0;
	goal_speed = goal_speed / (1.0 - deadband_value);

	double goal_speed2 = goal_speed * goal_speed;
	double goal_speed3 = goal_speed2 * goal_speed;
	double goal_speed4 = goal_speed3 * goal_speed;
	double goal_speed5 = goal_speed4 * goal_speed;
	double goal_speed6 = goal_speed5 * goal_speed;
	double goal_speed7 = goal_speed6 * goal_speed;

	// Constants for the 5th order polynomial
	double victor_fit_e1		= 0.437239;
	double victor_fit_c1		= -1.56847;
	double victor_fit_a1		= (- (125.0 * victor_fit_e1  + 125.0 * victor_fit_c1 - 116.0) / 125.0);
	double answer_5th_order = (victor_fit_a1 * goal_speed5
					+ victor_fit_c1 * goal_speed3
					+ victor_fit_e1 * goal_speed);

	// Constants for the 7th order polynomial
	double victor_fit_c2 = -5.46889;
	double victor_fit_e2 = 2.24214;
	double victor_fit_g2 = -0.042375;
	double victor_fit_a2 = (- (125.0 * (victor_fit_c2 + victor_fit_e2 + victor_fit_g2) - 116.0) / 125.0);
	double answer_7th_order = (victor_fit_a2 * goal_speed7
					+ victor_fit_c2 * goal_speed5
					+ victor_fit_e2 * goal_speed3
					+ victor_fit_g2 * goal_speed);


	// Average the 5th and 7th order polynomials
	double answer =  0.85 * 0.5 * (answer_7th_order + answer_5th_order)
			+ .15 * goal_speed * (1.0 - deadband_value);

	if (answer > 0.001)
		answer += deadband_value;
	else if (answer < -0.001)
		answer -= deadband_value;

	return answer;
}

void RobotState::Lock()
{
	semTake(m_lock,WAIT_FOREVER);
}

void RobotState::Unlock()
{
	semGive(m_lock);
}

