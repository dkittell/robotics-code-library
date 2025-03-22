package frc.robot;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// https://github.com/JonathanZwiebel/talon-srx-test/blob/master/CTREDrivetrain.java
// https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html

public class Robot extends TimedRobot {
  TalonSRX m_Climber01 = new TalonSRX(6);

  private final Joystick js1 = new Joystick(0);

  Faults _faults = new Faults(); /* temp to fill with latest faults */

  @Override
  public void robotInit() {

  }

  @Override
  public void teleopInit() {
    // TODO Auto-generated method stub
    super.teleopInit();

    /* factory default values */
    m_Climber01.configFactoryDefault();

    /*
     * choose whatever you want so "positive" values moves mechanism forward,
     * upwards, outward, etc.
     *
     * Note that you can set this to whatever you want, but this will not fix motor
     * output direction vs sensor direction.
     */
    m_Climber01.setInverted(false);

    /*
     * flip value so that motor output and sensor velocity are the same polarity. Do
     * this before closed-looping
     */
    m_Climber01.setSensorPhase(false); // <<<<<< Adjust this

  }

  @Override
  public void teleopPeriodic() {

    double xSpeed = 0.0; // make forward stick positive
    // double xSpeed = js1.getRawAxis(1) * -1; // make forward stick positive
    // double xSpeed = js1.getRawAxis(1) ; // make forward stick positive

    m_Climber01.getSelectedSensorPosition();
    xSpeed = -js1.getRawAxis(1) * -1; // make forward stick positive

   // double xSpeed = js1.getRawAxis(1) ; // make forward stick positive
    /* update motor controller */
    m_Climber01.set(ControlMode.PercentOutput, xSpeed);
    /* check our live faults */
    m_Climber01.getFaults(_faults);

    /* hold down btn1 to print stick values */
    // if (js1.getRawButton(1)) {
    SmartDashboard.putNumber("Sensor Vel", m_Climber01.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Sensor Pos", m_Climber01.getSelectedSensorPosition());
    SmartDashboard.putNumber("Out %", m_Climber01.getMotorOutputPercent());
    // SmartDashboard.putNumber("Out Of Phase",_faults.SensorOutOfPhase);
    // }

  }

  @Override
  public void disabledInit() {
    super.disabledInit();
  }

  @Override
  public void disabledPeriodic() {
    super.disabledPeriodic();
    /* factory default values */
    m_Climber01.configFactoryDefault();
    m_Climber01.setSelectedSensorPosition(0);

  }
}
