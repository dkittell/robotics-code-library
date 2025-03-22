#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <WPIlib.h>
//don't let it bark at you
class Robot: public frc::IterativeRobot {

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Robot Init~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

public:
	void RobotInit() {
		chooser.AddDefault(autoNameDefault, autoNameDefault);
		chooser.AddObject(autoNameCustom, autoNameCustom);
		frc::SmartDashboard::PutData("Auto Modes", &chooser);
		compressor->SetClosedLoopControl (true);

		CameraServer::GetInstance()->StartAutomaticCapture();
	}

	/*
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * GetString line to get the auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */

	void SetlDrive(float speed, float rotate){
		leftFront->Set (speed + rotate);
		leftRear->Set (speed + rotate);
	}

	void SetrDrive(float speed, float rotate){
	rightFront->Set (speed + rotate);
	rightRear->Set (speed + rotate);
	}

	void moveTheRobotsquared(float speed, float rotate){
		if (speed > 0.0) speed *= speed; speed = -(speed*speed);
		if (rotate > 0.0) rotate *= rotate; rotate = -(rotate*rotate);

		leftFront->Set (-speed + rotate);
		leftRear->Set (-speed + rotate);
		rightFront->Set (speed + rotate);
		rightRear->Set (speed + rotate);
	}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Autonomous Init~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
	void AutonomousInit() override {
	//	autoSelected = chooser.GetSelected();
		// std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
	//	std::cout << "Auto selected: " << autoSelected << std::endl;
	}
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Autonomous~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

	int autonState = 0;
	int autonCounter = 0;

	void AutonomousPeriodic() {
//		Wait(2);
//		SetlDrive(-1.0,-1.0);
//		SetrDrive(1.0,1.0);
//		Wait(1.5);
//		SetlDrive (0,0);
//		SetrDrive (0,0);
//		Wait(11.5);
	switch (selectedAuton){

		case 0:
			switch (autonState){
						case 0:
							if (autonCounter++ >= 100){	//wait 2 seconds
								autonState = 1;
							}
							// unpack the robot
							break;
						case 1:
							// move forward 10 feet
							SetlDrive(-1.0,-1.0);
							SetrDrive(1.0,1.0);
							autonCounter = 0;
							autonState = 2;
							break;
						case 2:
							if (autonCounter++ >= 100){			// wait 2 seconds
								SetlDrive (0,0);
								SetrDrive (0,0);
								autonState = 99;
							}
							break;
						default:
							break;
				}
			break;

		case 1:
			    switch (autonState){
						case 0:
							if (autonCounter++ >= 100){	//wait 2 seconds
								autonState = 1;
							}
							// unpack the robot
							break;
						case 1:
							// move forward 10 feet
							SetlDrive(-.15,-.15);
							SetrDrive(0.14,0.14);
							autonCounter = 0;
							autonState = 2;
							break;
						case 2:
							// wait for 10 feet to happen
							if (autonCounter++ >= 12){			// wait 1.5 seconds
							autonState = 3;
							}
							break;
						case 3:
							if (autonCounter++ >= 110){			// wait 1.5 seconds
							SetlDrive (0,0);
							SetrDrive (0,0);
							autonState = 4;
							}
							break;
						case 4:
							if (autonCounter++ >= 10){			// wait 1.5 seconds
							autonState = 5;
							}
							break;
						case 5:
							if (autonCounter++ >= 10){			// wait 1.5 seconds
							autonState = 99;
							}
							break;
						default:
							break;
					}
			break;
		case 2:
			break;
		case 3:
			break;
		default:
			break;
		}
	}
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Teleop~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
	void TeleopInit() {
		HopperState = 1;
		SpeedState = 0;
	}
		//	void TeleopPeriodic(const char* endl) {
		void TeleopPeriodic() {
		TankDrive (driverController->GetRawAxis(5), driverController->GetRawAxis(1));

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Wench~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

  	//This is the Wench to lift robot and climb rope
 		if (operatorController->GetRawButton (6)){
  			botWench->Set (1.0);
  		} else {
  			botWench->Set (0);
  		}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Hopper Stuff~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

  	// This is the hopper lift
		if (operatorController->GetRawButton(1)){
			hopperLift->Set (DoubleSolenoid::kForward);
		}else if (operatorController->GetRawButton(3)){
			hopperLift->Set (DoubleSolenoid::kReverse);
		}

	//	This is the hopper door
		if (HopperState == 1){
			hopperDoor->Set (DoubleSolenoid::kForward);
		} else if (HopperState == 2){
			hopperDoor->Set (DoubleSolenoid::kReverse);
		} else {
			// hopperDoor->Set (DoubleSolenoid::kOff);
		}

	//  This is the hopper state
		if (operatorController->GetRawButton(4)){
			HopperState = 1;
		} else if (operatorController->GetRawButton(2)){
			HopperState = 2;
		} else {
			HopperState = 3;
		}
	}
/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Tank Drive~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

	void TankDrive(double left, double right){
	std::cout <<"Left Stick: " << left << " Right Stick: "<< right << std::endl;

	if (SpeedState == 1){
		float speedScale = .65;
		left *= speedScale;
		right *= speedScale;
	} else if (SpeedState == 0){
		float speedScale = .99;
		left *= speedScale;
		right *= speedScale;
	}
	if (driverController->GetRawButton(5)){
		SpeedState = 1;
	}else {
		SpeedState = 0;
	}
		leftFront->Set (-right);
		leftRear->Set (-right);
		rightFront->Set (left);
		rightRear->Set (left);
	}

	int selectedAuton = 0;

	void DisabledPeriodic(){
		if (driverController->GetPOV(0) == 0) selectedAuton = 1;
		else if (driverController->GetPOV(0) == 90) selectedAuton = 2;
		else if (driverController->GetPOV(0) == 180) selectedAuton = 3;
		else if (driverController->GetPOV(0) == 270) selectedAuton = 0;

		std::cout << "Auton " << selectedAuton << " selected\n";
	}
	void TestPeriodic() {
		lw->Run();
	}

	/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Classes~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/


	/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Classes~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

private:
	frc::LiveWindow* lw = LiveWindow::GetInstance();
	frc::SendableChooser<std::string> chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "My Auto";
	std::string autoSelected;

	Compressor *compressor = new Compressor (0);

	Joystick *driverController = new Joystick (0);
	Joystick *operatorController = new Joystick (1);

   /*     from back of robot
     right Front motor into pwm 0
	 right  rear motor into pwm 1
	 left  front motor into pwm 2
	 right  rear motor into pwm 3 */

	//Drive Motors													
	Victor *rightFront = new Victor (0);
	Victor *rightRear = new Victor (1);
	Victor *leftFront = new Victor (2);								
	Victor *leftRear = new Victor (3);

	// Robot Abilities
	Victor *botWench = new Victor (6);

    //	Solenoids
	DoubleSolenoid *hopperDoor = new DoubleSolenoid (0, 1);
	DoubleSolenoid *hopperLift = new DoubleSolenoid (2, 3);

	// Hopper State
	int HopperState = 1;
	int SpeedState = 0;
};

START_ROBOT_CLASS (Robot);