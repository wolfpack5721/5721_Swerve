/*
 * Test of swerve drive using libarary from team 2081
 * created 04/19/2017  Chester Marshall
 *
 * 04/16/2017 -  testing Turn Motor control using incremental encoder
 * 04/25/2017 -  implement for all four wheels, ready to test on bot
 *
 *
 *
 *   LF DRV-PWM 2                  RF DRV-PWM 0
 *   LF TRN-PWM 3                  RF TRN-PWM 1
 *   LF ENC-DIO 2,3                RF ENC-DIO 0,1
 *
 *
 *   LR DRV-PWM 4                  RR DRV-PWM 6
 *   LR TRN-PWM 5                  RR TRN-PWM 7
 *   LR ENC-DIO 4,5                RR ENC-DIO 6,7
 *
 *
 */

#include "swervelib.h"
#include "wpilib.h"

using namespace frc;

const static double kP = 0.08f;
const static double kI = 0.00f;
const static double kD = 0.006f;
const static double kF = 0.00f;

//wrapper class used to protect input to pidcontroller
class SafeEncoder : public PIDSource {
private:
	Encoder* m_safeEncoder;
public:
	SafeEncoder(Encoder *regEncoder) {
		m_safeEncoder = regEncoder;
  	}
	virtual ~SafeEncoder() = default;
	double PIDGet() override {
		if(m_safeEncoder->GetDistance() < 0)
			return 0;
		if(m_safeEncoder->GetDistance() > 360)
			return 360;
		return m_safeEncoder->GetDistance();
	}
	void SetPIDSourceType(PIDSourceType pidSource) override	{
		if (wpi_assert(pidSource == PIDSourceType::kDisplacement))
			m_pidSource = pidSource;
	}
};


class Robot: public IterativeRobot
{
private:
	LiveWindow* LW = LiveWindow::GetInstance();
	VictorSP *MtrDriveRF;
	VictorSP *MtrTurnRF;
	Encoder  *EncTurnRF;
	SafeEncoder *EncTurnRFs;
	PIDController *PidRF;
	VictorSP *MtrDriveLF;
	VictorSP *MtrTurnLF;
	Encoder  *EncTurnLF;
	PIDController *PidLF;
	VictorSP *MtrDriveLR;
	VictorSP *MtrTurnLR;
	Encoder  *EncTurnLR;
	PIDController *PidLR;
	VictorSP *MtrDriveRR;
	VictorSP *MtrTurnRR;
	Encoder  *EncTurnRR;
	PIDController *PidRR;
	swervelib *Swerve;
	Joystick *StickLeft;
	Joystick *StickRight;
	Preferences *Prefs;
	double JoystickLeftX = 0;
	double JoystickLeftY = 0;
	double JoystickRightX = 0;

public:
	void RobotInit()
	{
		printf("[RobotInit]\n");
		//PG71 is 497/1988 = 360/1988 = .18109
		double distPerPulse = .18109;

		//Right Front
		MtrDriveRF = new VictorSP(0);
		MtrTurnRF = new VictorSP(1);
		EncTurnRF = new Encoder(0,1, false, Encoder::k4X);
		EncTurnRF->SetDistancePerPulse(distPerPulse);
		EncTurnRF->SetPIDSourceType(PIDSourceType::kDisplacement);
		EncTurnRFs = new SafeEncoder(EncTurnRF);
		PidRF = new PIDController(kP,kI,kD,kF, EncTurnRF, MtrTurnRF, 0.02);
		//PidRF = new PIDController(kP,kI,kD,kF, EncTurnRFs, MtrTurnRF, 0.02);
		PidRF->SetInputRange(0,360);
		PidRF->SetOutputRange(-1,1);
		PidRF->SetContinuous(true);
		PidRF->Enable();
		//Left Front
		MtrDriveLF = new VictorSP(2);
		MtrTurnLF = new VictorSP(3);
		EncTurnLF = new Encoder(2,3, false, Encoder::k4X);
		EncTurnLF->SetDistancePerPulse(distPerPulse);
		EncTurnLF->SetPIDSourceType(PIDSourceType::kDisplacement);
		PidLF = new PIDController(kP,kI,kD,kF, EncTurnRF, MtrTurnRF, 0.02);
		PidLF->SetInputRange(0,360);
		PidLF->SetOutputRange(-1,1);
		PidLF->SetContinuous(true);
		PidLF->Enable();
		//Left Rear
		MtrDriveLR = new VictorSP(4);
		MtrTurnLR = new VictorSP(5);
		EncTurnLR = new Encoder(4,5, false, Encoder::k4X);
		EncTurnLR->SetDistancePerPulse(distPerPulse);
		EncTurnLR->SetPIDSourceType(PIDSourceType::kDisplacement);
		PidLR = new PIDController(kP,kI,kD,kF, EncTurnRF, MtrTurnRF, 0.02);
		PidLR->SetInputRange(0,360);
		PidLR->SetOutputRange(-1,1);
		PidLR->SetContinuous(true);
		PidLR->Enable();
		//Right Rear
		MtrDriveRR = new VictorSP(6);
		MtrTurnRR = new VictorSP(7);
		EncTurnRR = new Encoder(6,7, false, Encoder::k4X);
		EncTurnRR->SetDistancePerPulse(distPerPulse);
		EncTurnRR->SetPIDSourceType(PIDSourceType::kDisplacement);
		PidRR = new PIDController(kP,kI,kD,kF, EncTurnRF, MtrTurnRF, 0.02);
		PidRR->SetInputRange(0,360);
		PidRR->SetOutputRange(-1,1);
		PidRR->SetContinuous(true);
		PidRR->Enable();

		Swerve = new swervelib(36,36); //3ft wide x 3ft long
		StickLeft = new Joystick(0);
		StickRight = new Joystick(1);

		Prefs = Preferences::GetInstance();
		std::vector <std::string> allKeys = Prefs->GetKeys();
		for(unsigned int x=0; x<allKeys.size(); x++)
			Prefs->Remove(allKeys[x]);
		Prefs->PutDouble("Turn_P",kP);
		Prefs->PutDouble("Turn_I",kI);
		Prefs->PutDouble("Turn_D",kD);
	}

	void AutonomousInit() override
	{
		printf("[AutonomousInit]\n");
	}

	void AutonomousPeriodic()
	{
		JoystickLeftX = StickLeft->GetRawAxis(0);
		JoystickLeftY = StickLeft->GetRawAxis(1);
		SmartDashboard::PutNumber("JoyLeftX",JoystickLeftX);
		SmartDashboard::PutNumber("JoyLeftY",JoystickLeftY);
		SmartDashboard::PutNumber("JoyRightX",JoystickRightX);
		SmartDashboard::PutNumber("TurnRF_SP",PidRF->GetSetpoint());
		SmartDashboard::PutNumber("TurnRF_PV",PidRF->GetSetpoint() - PidRF->GetError());
		SmartDashboard::PutNumber("TurnRF_PO",PidRF->Get());
		SmartDashboard::PutNumber("DriveRF_Speed",Swerve->whl->speed1);

	}

	void TeleopInit()
	{
		printf("[TeleopInit]\n");
		PidRF->SetPID(Prefs->GetDouble("Turn_P"),Prefs->GetDouble("Turn_I"),Prefs->GetDouble("Turn_D"));
		PidLF->SetPID(Prefs->GetDouble("Turn_P"),Prefs->GetDouble("Turn_I"),Prefs->GetDouble("Turn_D"));
		PidLR->SetPID(Prefs->GetDouble("Turn_P"),Prefs->GetDouble("Turn_I"),Prefs->GetDouble("Turn_D"));
		PidRR->SetPID(Prefs->GetDouble("Turn_P"),Prefs->GetDouble("Turn_I"),Prefs->GetDouble("Turn_D"));
	}

	void TeleopPeriodic()
	{
		//get joystick positions
		JoystickLeftX = StickLeft->GetRawAxis(0);
		JoystickLeftY = StickLeft->GetRawAxis(1);
		JoystickRightX = 0; //StickRight->GetRawAxis(0);  //rudder control

		//calculate wheel angles and speeds
		Swerve->calcWheelVect(JoystickLeftX, JoystickLeftY, JoystickRightX);

		//set wheel turning motor PID loop setpoint to follow new angle
		//feedback for turning is from motor encoder directly to PIDcontroller object
		//output to turning motor is made by PIDcontroller object
		PidRF->SetSetpoint(Swerve->whl->angle1);
		PidLF->SetSetpoint(Swerve->whl->angle2);
		PidLR->SetSetpoint(Swerve->whl->angle3);
		PidRR->SetSetpoint(Swerve->whl->angle4);

		//set wheel driving motor speed
		MtrDriveRF->Set(Swerve->whl->speed1);
		MtrDriveLF->Set(Swerve->whl->speed2);
		MtrDriveLR->Set(Swerve->whl->speed3);
		MtrDriveRR->Set(Swerve->whl->speed4);

		//output data to dashboard
		SmartDashboard::PutNumber("JoyLeftX",JoystickLeftX);
		SmartDashboard::PutNumber("JoyLeftY",JoystickLeftY);
		SmartDashboard::PutNumber("JoyRightX",JoystickRightX);
		SmartDashboard::PutNumber("TurnRF_SP",PidRF->GetSetpoint());
		SmartDashboard::PutNumber("TurnRF_PV",PidRF->GetSetpoint() - PidRF->GetError());
		SmartDashboard::PutNumber("TurnRF_PO",PidRF->Get());
		SmartDashboard::PutNumber("DriveRF_Speed",Swerve->whl->speed1);
	}

	void TestInit() override
	{
		printf("[TestInit]\n");
	}

	void TestPeriodic()
	{
		LW->Run();
	}

	void DisabledInit() override
	{
		printf("[Robot was DISABLED]\n");
	}
};

START_ROBOT_CLASS(Robot)
