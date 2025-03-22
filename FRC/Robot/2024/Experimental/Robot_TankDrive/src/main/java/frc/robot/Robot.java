// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// CTRE - https://maven.ctr-electronics.com/release/com/ctre/phoenix/Phoenix-frc2024-latest.json
// Rev Robotics - https://software-metadata.revrobotics.com/REVLib-2024.json
// Venom - https://www.playingwithfusion.com/frc/playingwithfusion2024.json
// NavX Library - https://dev.studica.com/releases/2024/NavX.json
// import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.constants;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  public String kDefaultAuto = "Default";
  public String kCustomAuto = "My Auto";
  SendableChooser<String> m_chooser = new SendableChooser<>();
  String m_autoSelected;

  DifferentialDrive m_robotDrive;
  Joystick js_Driver;
  Joystick js_Operator;

  double max_drive_speed;
  double max_turn_speed;

  // Ramp the speed of the drive motors
  SlewRateLimiter driveX = new SlewRateLimiter(0.6);
  SlewRateLimiter driveY = new SlewRateLimiter(1);
  SlewRateLimiter driveZ = new SlewRateLimiter(1);

  I2C.Port i2cPort = I2C.Port.kOnboard;

  ColorSensorV3 cs_Conveyor = new ColorSensorV3(i2cPort);

  CANSparkMax m_Conveyor = new CANSparkMax(
    constants.can_id_Conveyor,
    MotorType.kBrushless
  );
  CANSparkMax m_Intake = new CANSparkMax(
    constants.can_id_intake,
    MotorType.kBrushed
  );
  CANSparkMax m_Shooter1 = new CANSparkMax(
    constants.can_id_Shooter1,
    MotorType.kBrushed
  );
  CANSparkMax m_Shooter2 = new CANSparkMax(
    constants.can_id_Shooter2,
    MotorType.kBrushed
  );
  CANSparkMax m_leftMotor = new CANSparkMax(
    constants.can_id_frontLeft,
    MotorType.kBrushed
  );
  CANSparkMax m_rightMotor = new CANSparkMax(
    constants.can_id_frontRight,
    MotorType.kBrushed
  );

  public AHRS ahrs;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    SendableRegistry.addChild(m_robotDrive, m_leftMotor);
    SendableRegistry.addChild(m_robotDrive, m_rightMotor);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);
    m_Shooter1.setInverted(true);
    m_Shooter2.setInverted(true);

    m_robotDrive = new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);
    js_Driver = new Joystick(constants.js_Driver);
    js_Operator = new Joystick(constants.js_Operator);

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
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    Color detectedColor = cs_Conveyor.getColor();
    double IR = cs_Conveyor.getIR();

    /**
     * Open Smart Dashboard or Shuffleboard to see the color detected by the
     * sensor.
     */
    SmartDashboard.putNumber("Red", cs_Conveyor.getRed());
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("IR", IR);

    SmartDashboard.putNumber("Max Drive", max_drive_speed);
    SmartDashboard.putNumber("Max Turn", max_turn_speed);
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // switch (m_autoSelected) {
    //   case kCustomAuto:
    //     // Put custom auto code here
    //     break;
    //   case kDefaultAuto:
    //   default:
    //     // Put default auto code here
    //     break;
    // }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    max_drive_speed = 0.80;
    max_turn_speed = 0.60;
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // m_robotDrive.tankDrive(js_Driver.getRawAxis(1), js_Driver.getRawAxis(2));
    m_robotDrive.arcadeDrive(
      js_Driver.getRawAxis(constants.axisX_lUpDown),
      js_Driver.getRawAxis(constants.axisX_rLeftRight)
    );

    if (js_Operator.getRawButton(constants.btnD_RB)) {
      // Bring note to conveyor
      m_Intake.set(0.7);
    } else if (js_Operator.getRawButton(constants.btnD_RT)) {
      // Kick the note out of robot
      m_Intake.set(-0.7);
    } else {
      m_Intake.set(0);
    }

    if (js_Operator.getRawButton(constants.btnD_A)) {
      // Kick the note back to the conveyor
      m_Shooter1.set(1);
      m_Shooter2.set(1);
    } else if (js_Operator.getRawButton(constants.btnD_X)) {
      // Shoot the note
      m_Shooter1.set(-1);
      m_Shooter2.set(-1);
    } else if (js_Operator.getRawButton(constants.btnD_B)) {
      m_Shooter1.set(-0.8);
      m_Shooter2.set(-0.4);
    } else {
      m_Shooter1.set(0);
      m_Shooter2.set(0);
    }
    if (cs_Conveyor.getRed() > 0.5) {
      m_Conveyor.set(0);
    }

    if (js_Operator.getRawButton(constants.btnD_LB)) {
      // Bring note to shooter
      m_Conveyor.set(1);
    } else if (js_Operator.getRawButton(constants.btnD_LT)) {
      // Kick the note back to intake
      m_Conveyor.set(-1);
    } else {
      m_Conveyor.set(0);
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    /* Display 6-axis Processed Angle Data                                      */
    SmartDashboard.putBoolean("IMU_Connected", ahrs.isConnected());
    SmartDashboard.putBoolean("IMU_IsCalibrating", ahrs.isCalibrating());
    SmartDashboard.putNumber("IMU_Yaw", ahrs.getYaw());
    SmartDashboard.putNumber("IMU_Pitch", ahrs.getPitch());
    SmartDashboard.putNumber("IMU_Roll", ahrs.getRoll());

    /* Display tilt-corrected, Magnetometer-based heading (requires             */
    /* magnetometer calibration to be useful)                                   */

    SmartDashboard.putNumber("IMU_CompassHeading", ahrs.getCompassHeading());

    /* Display 9-axis Heading (requires magnetometer calibration to be useful)  */
    SmartDashboard.putNumber("IMU_FusedHeading", ahrs.getFusedHeading());

    /* These functions are compatible w/the WPI Gyro Class, providing a simple  */
    /* path for upgrading from the Kit-of-Parts gyro to the navx-MXP            */

    // SmartDashboard.putNumber("IMU_TotalYaw", ahrs.getAngle());
    // SmartDashboard.putNumber("IMU_YawRateDPS", ahrs.getRate());

    /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */

    // SmartDashboard.putNumber("IMU_Accel_X", ahrs.getWorldLinearAccelX());
    // SmartDashboard.putNumber("IMU_Accel_Y", ahrs.getWorldLinearAccelY());
    SmartDashboard.putBoolean("IMU_IsMoving", ahrs.isMoving());
    SmartDashboard.putBoolean("IMU_IsRotating", ahrs.isRotating());

    /* Display estimates of velocity/displacement.  Note that these values are  */
    /* not expected to be accurate enough for estimating robot position on a    */
    /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
    /* of these errors due to single (velocity) integration and especially      */
    /* double (displacement) integration.                                       */

    // SmartDashboard.putNumber("Velocity_X", ahrs.getVelocityX());
    // SmartDashboard.putNumber("Velocity_Y", ahrs.getVelocityY());
    // SmartDashboard.putNumber("Displacement_X", ahrs.getDisplacementX());
    // SmartDashboard.putNumber("Displacement_Y", ahrs.getDisplacementY());

    /* Display Raw Gyro/Accelerometer/Magnetometer Values                       */
    /* NOTE:  These values are not normally necessary, but are made available   */
    /* for advanced users.  Before using this data, please consider whether     */
    /* the processed data (see above) will suit your needs.                     */

    // SmartDashboard.putNumber("RawGyro_X", ahrs.getRawGyroX());
    // SmartDashboard.putNumber("RawGyro_Y", ahrs.getRawGyroY());
    // SmartDashboard.putNumber("RawGyro_Z", ahrs.getRawGyroZ());
    // SmartDashboard.putNumber("RawAccel_X", ahrs.getRawAccelX());
    // SmartDashboard.putNumber("RawAccel_Y", ahrs.getRawAccelY());
    // SmartDashboard.putNumber("RawAccel_Z", ahrs.getRawAccelZ());
    // SmartDashboard.putNumber("RawMag_X", ahrs.getRawMagX());
    // SmartDashboard.putNumber("RawMag_Y", ahrs.getRawMagY());
    // SmartDashboard.putNumber("RawMag_Z", ahrs.getRawMagZ());
    SmartDashboard.putNumber("IMU_Temp_C", ahrs.getTempC());

    /* Omnimount Yaw Axis Information                                           */
    /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount  */
    // AHRS.BoardYawAxis yaw_axis = ahrs.getBoardYawAxis();
    // SmartDashboard.putString("YawaxisXirection", yaw_axis.up ? "Up" : "Down");
    // SmartDashboard.putNumber("YawAxis", yaw_axis.board_axis.getValue());

    /* Sensor Board Information                                                 */
    SmartDashboard.putString("FirmwareVersion", ahrs.getFirmwareVersion());

    // /* Quaternion Data                                                          */
    // /* Quaternions are fascinating, and are the most compact representation of  */
    // /* orientation data.  All of the Yaw, Pitch and Roll Values can be derived  */
    // /* from the Quaternions.  If interested in motion processing, knowledge of  */
    // /* Quaternions is highly recommended.                                       */
    // SmartDashboard.putNumber("QuaternionW", ahrs.getQuaternionW());
    // SmartDashboard.putNumber("QuaternionX", ahrs.getQuaternionX());
    // SmartDashboard.putNumber("QuaternionY", ahrs.getQuaternionY());
    // SmartDashboard.putNumber("QuaternionZ", ahrs.getQuaternionZ());

    // /* Connectivity Debugging Support                                           */
    // SmartDashboard.putNumber("IMU_Byte_Count", ahrs.getByteCount());
    // SmartDashboard.putNumber("IMU_Update_Count", ahrs.getUpdateCount());

    // If the ADC Value is Above 200 then...
    if (cs_Conveyor.getRed() > 200) {
      // This checks if the shooter buttons are pressed to override the color sensor
      if (js_Operator.getRawAxis(constants.axisX_RTrigger) > 0.17) {
        // basic conveyer runner
        if (js_Operator.getRawButton(constants.btnD_LB)) {
          // Bring note to shooter
          m_Conveyor.set(1);
        } else if (js_Operator.getRawButton(constants.btnD_X)) {
          // Kick the note back to intake
          m_Conveyor.set(-1);
        } else {
          m_Conveyor.set(0);
        }
      }
      // this else is if the color sensor has not detected anything
    } else {
      if (js_Operator.getRawButton(constants.btnD_LB)) {
        // Bring note to shooter
        m_Conveyor.set(0.6);
      } else if (js_Operator.getRawButton(constants.btnD_X)) {
        // Kick the note back to intake
        m_Conveyor.set(-1);
      } else {
        m_Conveyor.set(0);
      }
      if (js_Operator.getRawButton(constants.btnD_RB)) {
        // Bring note to conveyor
        m_Intake.set(0.7);
      } else if (js_Operator.getRawButton(constants.btnD_B)) {
        // Kick the note out of robot
        m_Intake.set(-0.7);
      } else {
        m_Intake.set(0);
      }
    }
    // if (js_Operator.getRawButton(5)) {
    // m_Conveyor.set(0.7);
    // } else if (js_Operator.getRawButton(7)) {
    // m_Conveyor.set(-0.7);
    // } else {
    // m_Conveyor.set(0);
    // }

    if (js_Driver.getRawButtonPressed(constants.btnD_X)) {
      if (max_drive_speed < 1) {
        max_drive_speed = max_drive_speed + 0.10;
      } else {
        max_drive_speed = 1;
      }
    } else if (js_Driver.getRawButtonPressed(constants.btnD_A)) {
      if (max_drive_speed > 0.1) {
        max_drive_speed = max_drive_speed - 0.10;
      } else {
        max_drive_speed = 0.1;
      }
    }
    if (js_Driver.getRawButtonPressed(constants.btnD_Y)) {
      if (max_turn_speed < 1) {
        max_turn_speed = max_turn_speed + 0.10;
      } else {
        max_turn_speed = 1;
      }
    } else if (js_Driver.getRawButtonPressed(constants.btnD_B)) {
      if (max_turn_speed > 0.1) {
        max_turn_speed = max_turn_speed - 0.10;
      } else {
        max_turn_speed = 0.1;
      }
    }

    // Limits the speed to 80% of the joystick
    // TO DO: Check negatives on driver axis
    double speedY =
      -js_Driver.getRawAxis(constants.axisD_lUpDown) * (max_drive_speed);
    double speedZ =
      js_Driver.getRawAxis(constants.axisD_rLeftRight) * (max_turn_speed);

    m_robotDrive.arcadeDrive(
     driveY.calculate(speedY),
    driveZ.calculate(speedZ)
    );

    // if (js_Driver.getRawButton(constants.btnD_RB)) {
    // // Bring note to conveyor
    // m_Intake.set(0.7);
    // } else if (js_Driver.getRawButton(constants.btnD_RT)) {
    // // Kick the note out of robot
    // m_Intake.set(-0.7);
    // } else {
    // m_Intake.set(0);
    // }

    // if (js_Driver.getRawButton(constants.axisX_LTrigger)) {
    //   // Kick the note back to the conveyor
    //   m_Shooter1.set(0.5);
    //   m_Shooter2.set(0.5);
    // } else if (js_Driver.getRawButton(constants.axisX_RTrigger)) {
    //   // Shoot the note
    //   m_Shooter1.set(-1);
    //   m_Shooter2.set(-1);
    // } else if (js_Driver.getRawButton(constants.btnD_B)) {
    //   m_Shooter1.set(-0.5);
    //   m_Shooter2.set(-0.6);
    // } else {
    //   m_Shooter1.set(0);
    //   m_Shooter2.set(0);
    // }
    m_Shooter1.set(js_Operator.getRawAxis(constants.axisX_LTrigger));
    m_Shooter2.set(js_Operator.getRawAxis(constants.axisX_RTrigger));

    if (js_Operator.getRawAxis(constants.axisX_LTrigger) > 0.17) {
      m_Shooter1.set(-0.4);
      m_Shooter2.set(-0.4);
    }

    if (js_Driver.getRawButton(constants.btnD_Start)) {
      ahrs.reset();
    }

    boolean zero_yaw_pressed = js_Driver.getRawButton(constants.btnD_Back);
    if (zero_yaw_pressed) {
      ahrs.zeroYaw();
    }
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
