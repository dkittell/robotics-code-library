package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Servo;

// Motor Controllers
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

// https://maven.ctr-electronics.com/release/com/ctre/phoenix/Phoenix-frc2022-latest.json
import com.ctre.phoenix.motorcontrol.can.*;

public class Robot extends TimedRobot {

  Servo _ratchetServo = new Servo(2);

  WPI_TalonSRX _talonL = new WPI_TalonSRX(8);
  WPI_TalonSRX _talonR = new WPI_TalonSRX(9);
  DifferentialDrive _drive = new DifferentialDrive(_talonL, _talonR);
  Joystick _joystick = new Joystick(1);

  @Override
  public void teleopInit() {
    /* factory default values */
    _talonL.configFactoryDefault();
    _talonR.configFactoryDefault();

    /* flip values so robot moves forward when stick-forward/LEDs-green */
    _talonL.setInverted(false); // <<<<<< Adjust this
    _talonR.setInverted(false); // <<<<<< Adjust this
  }

  @Override
  public void teleopPeriodic() {
    double xSpeed = _joystick.getRawAxis(1) * -1; // make forward stick positive
    double zRotation = _joystick.getRawAxis(2); // WPI Drivetrain uses positive=> right

    _drive.arcadeDrive(xSpeed, 0);
    // _drive.arcadeDrive(xSpeed, zRotation);

    if (_joystick.getRawButtonPressed(1)) {
      _ratchetServo.set(1.0); 
    }
    if (_joystick.getRawButtonReleased(1)) {
      _ratchetServo.set(0.0);
    }

    /* hold down btn1 to print stick values */
    if (_joystick.getRawButton(1)) {
      System.out.println("xSpeed:" + xSpeed + "    zRotation:" + zRotation);
    }
  }
}