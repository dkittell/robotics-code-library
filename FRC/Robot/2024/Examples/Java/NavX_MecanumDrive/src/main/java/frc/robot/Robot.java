// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS; // https://dev.studica.com/releases/2024/NavX.json
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

/** This is a demo program showing how to use Mecanum control with the MecanumDrive class. */
public class Robot extends TimedRobot {

  AHRS ahrs;
  Joystick m_stick;
  MecanumDrive m_robotDrive;
  int kFrontLeftChannel = 2;
  int kFrontRightChannel = 1;
  int kJoystickChannel = 0;
  int kRearLeftChannel = 3;
  int kRearRightChannel = 0;

  @Override
  public void robotInit() {
    PWMSparkMax frontLeft = new PWMSparkMax(kFrontLeftChannel);
    PWMSparkMax rearLeft = new PWMSparkMax(kRearLeftChannel);
    PWMSparkMax frontRight = new PWMSparkMax(kFrontRightChannel);
    PWMSparkMax rearRight = new PWMSparkMax(kRearRightChannel);

    SendableRegistry.addChild(m_robotDrive, frontLeft);
    SendableRegistry.addChild(m_robotDrive, rearLeft);
    SendableRegistry.addChild(m_robotDrive, frontRight);
    SendableRegistry.addChild(m_robotDrive, rearRight);

    // Invert the right side motors.
    frontRight.setInverted(true);
    rearRight.setInverted(true);

    m_robotDrive =
      new MecanumDrive(
        frontLeft::set,
        rearLeft::set,
        frontRight::set,
        rearRight::set
      );
    m_robotDrive.setExpiration(0.1);

    m_stick = new Joystick(kJoystickChannel);

    try {
      /***********************************************************************
       * navX-MXP:
       * - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB.
       * - See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
       *
       * navX-Micro:
       * - Communication via I2C (RoboRIO MXP or Onboard) and USB.
       * - See http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
       *
       * Multiple navX-model devices on a single robot are supported.
       ************************************************************************/
      ahrs = new AHRS(SPI.Port.kMXP);
    } catch (RuntimeException ex) {
      DriverStation.reportError(
        "Error instantiating navX MXP:  " + ex.getMessage(),
        true
      );
    }
  }

  /**
   * Drive left & right motors for 2 seconds then stop
   */
  @Override
  public void autonomousPeriodic() {
    m_robotDrive.setSafetyEnabled(false);
    m_robotDrive.driveCartesian(0.0, -0.5, 0.0); // drive forwards half speed
    Timer.delay(2.0); //  for 2 seconds
    m_robotDrive.driveCartesian(0.0, 0.0, 0.0); // stop robot
  }

  @Override
  public void teleopPeriodic() {
    // Use the joystick Y axis for forward movement, X axis for lateral
    // movement, and Z axis for rotation.
    // m_robotDrive.driveCartesian(
    //   -m_stick.getY(),
    //   -m_stick.getX(),
    //   -m_stick.getZ()
    // );

    m_robotDrive.setSafetyEnabled(true);

    if (m_stick.getRawButton(1)) {
      ahrs.reset();
    }
    try {
      /* Use the joystick X axis for lateral movement,            */
      /* Y axis for forward movement, and Z axis for rotation.    */
      /* Use navX MXP yaw angle to define Field-centric transform */
      m_robotDrive.driveCartesian(
        m_stick.getX(),
        m_stick.getY(),
        m_stick.getTwist()
      );
    } catch (RuntimeException ex) {
      DriverStation.reportError(
        "Error communicating with drive system:  " + ex.getMessage(),
        true
      );
    }
    Timer.delay(0.005); // wait for a motor update time
  }
}
