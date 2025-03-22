package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name = "Mark_Auton02_Gyro_SquareB (Java)", group = "")
public class Mark_Auton02_Gyro_SquareB extends LinearOpMode {
  private AndroidTextToSpeech androidTextToSpeech;
  private DcMotor FrontLeft;
  private DcMotor FrontRight;
  private DcMotor BackLeft;
  private DcMotor BackRight;
  private BNO055IMU imu;

  /**
   * This function is executed when this Op Mode is selected from the Driver
   * Station.
   */
  @Override
  public void runOpMode() {
    double mP;
    double tF;
    double tB;
    double tLS;
    double tRS;
    BNO055IMU.Parameters IMU_Parameters;
    ElapsedTime ElapsedTime2;
    double Left_Power;
    double Right_Power;
    double Yaw_Angle;

    androidTextToSpeech = new AndroidTextToSpeech();
    FrontLeft = hardwareMap.get(DcMotor.class, "Front Left");
    FrontRight = hardwareMap.get(DcMotor.class, "Front Right");
    BackLeft = hardwareMap.get(DcMotor.class, "Back Left");
    BackRight = hardwareMap.get(DcMotor.class, "Back Right");
    imu = hardwareMap.get(BNO055IMU.class, "imu");

    // Motor Power Variable
    mP = 0.8;
    // Driving Encoder Ticks
    // tF - Ticks Forward
    tF = 3000;
    // tB - TIcks Backward
    tB = 2900;
    // tLS - Ticks Left Strafe
    tLS = 4100;
    // tRS - Ticks Right Strafe
    tRS = 3400;
    // Initialize Text2Speech
    androidTextToSpeech.initialize();
    androidTextToSpeech.setLanguageAndCountry("en", "US");
    // This op mode uses the REV Hub's built-in gyro to
    // to allow a robot to move straight forward and then
    // make a right turn.
    // The op mode assume you have
    // (1) Connected two motors to the expansion
    // hub.
    // (2) Created a config file that
    // (a) names the motors "left-motor" and
    // "right-motor"
    // (b) configures the imu on I2C bus 0 port 0
    // as a REV Expansion Hub IMU
    // with the name "imu".
    // Setup so motors will brake the wheels
    // when motor power is set to zero.
    FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    // Reverse direction of one motor so robot moves
    // forward rather than spinning in place.
    FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
    BackRight.setDirection(DcMotorSimple.Direction.REVERSE);
    // Create an IMU parameters object.
    IMU_Parameters = new BNO055IMU.Parameters();
    // Set the IMU sensor mode to IMU. This mode uses
    // the IMU gyroscope and accelerometer to
    // calculate the relative orientation of hub and
    // therefore the robot.
    IMU_Parameters.mode = BNO055IMU.SensorMode.IMU;
    // Intialize the IMU using parameters object.
    imu.initialize(IMU_Parameters);
    // Report the initialization to the Driver Station.
    telemetry.addData("Status", "IMU initialized, calibration started.");
    telemetry.update();
    // Wait one second to ensure the IMU is ready.
    sleep(1000);
    // Loop until IMU has been calibrated.
    while (!IMU_Calibrated()) {
      telemetry.addData("If calibration ",
          "doesn't complete after 3 seconds, move through 90 degree pitch, roll and yaw motions until calibration complete ");
      telemetry.update();
      // Wait one second before checking calibration
      // status again.
      sleep(1000);
    }
    // Report calibration complete to Driver Station.
    telemetry.addData("Status", "Calibration Complete");
    telemetry.addData("Action needed:", "Please press the start triangle");
    telemetry.update();
    // Wait for Start to be pressed on Driver Station.
    waitForStart();
    // Create a timer object with millisecond
    // resolution and save in ElapsedTime variable.
    ElapsedTime2 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    // Initialize motor power variables to 30%.
    Left_Power = 0.3;
    Right_Power = 0.3;
    // Set motor powers to the variable values.
    androidTextToSpeech.speak("I will move forward 1");
    telemetry.addData("Direction", "Forward");
    telemetry.update();
    FrontLeft.setPower(Left_Power);
    FrontRight.setPower(Right_Power);
    BackLeft.setPower(Left_Power);
    BackRight.setPower(Right_Power);
    // Move robot forward for 2 seconds or until stop
    // is pressed on Driver Station.
    while (!(ElapsedTime2.milliseconds() >= 2000 || isStopRequested())) {
      // Save gyro's yaw angle
      Yaw_Angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
      // Report yaw orientation to Driver Station.
      telemetry.addData("Yaw angle", Yaw_Angle);
      // If the robot is moving straight ahead the
      // yaw value will be close to zero. If it's not, we
      // need to adjust the motor powers to adjust heading.
      // If robot yaws right or left by 5 or more,
      // adjust motor power variables to compensation.
      if (Yaw_Angle < -5) {
        // Turn left
        Left_Power = 0.25;
        Right_Power = 0.35;
      } else if (Yaw_Angle > 5) {
        // Turn right.
        Left_Power = 0.35;
        Right_Power = 0.25;
      } else {
        // Continue straight
        Left_Power = 0.3;
        Right_Power = 0.3;
      }
      // Report the new power levels to the Driver Station.
      telemetry.addData("Left Motor Power", Left_Power);
      telemetry.addData("Right Motor Power", Right_Power);
      // Update the motors to the new power levels.
      FrontLeft.setPower(Left_Power);
      FrontRight.setPower(Right_Power);
      BackLeft.setPower(Left_Power);
      BackRight.setPower(Right_Power);
      telemetry.update();
      // Wait 1/5 second before checking again.
      sleep(200);
    }
    // Now let's execute a left turn using power
    // levels that will cause a turn in place.
    androidTextToSpeech.speak("I will turn left 1");
    telemetry.addData("Direction", "Left");
    telemetry.update();
    FrontLeft.setPower(-0.2);
    FrontRight.setPower(0.2);
    BackLeft.setPower(-0.2);
    BackRight.setPower(0.2);
    // Continue until robot yaws left by 90 degrees
    // or stop is pressed on Driver Station.

    Yaw_Angle = -imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

    while (!(Yaw_Angle <= -90 || isStopRequested())) {
      // Update Yaw-Angle variable with current yaw.
      Yaw_Angle = -imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
      // Report yaw orientation to Driver Station.
      telemetry.addData("Yaw Angle", Yaw_Angle);
      telemetry.update();
    }
    // Reset Encoders: All Motors
    FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    // Set TargetPosition and ticks
    // Make mark move forward
    telemetry.update();
    telemetry.update();
    // We're done. Turn off motors
    androidTextToSpeech.speak("I am done");
    FrontLeft.setPower(0);
    FrontRight.setPower(0);
    BackLeft.setPower(0);
    BackRight.setPower(0);
    // Pause so final telemetry is displayed.
    sleep(1000);

    androidTextToSpeech.close();
  }

  /**
   * Function that becomes true when gyro is calibrated and reports calibration
   * status to Driver Station in the meantime.
   */
  private boolean IMU_Calibrated() {
    telemetry.addData("IMU Calibration Status", imu.getCalibrationStatus());
    telemetry.addData("Gyro Calibrated", imu.isGyroCalibrated() ? "True" : "False");
    telemetry.addData("System Status", imu.getSystemStatus().toString());
    return imu.isGyroCalibrated();
  }
}
