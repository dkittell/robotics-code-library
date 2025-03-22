package org.firstinspires.ftc.teamcode.Teleop;

//region Imports
import android.graphics.Color;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.lang.Math;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//endregion Imports

@TeleOp(name = "Scooter Driving 2024", group = "Linear Opmode")
public class Scooter_Driving_2024 extends LinearOpMode {

  //region Initial Constants
  private DcMotor mFrontLeft;
  private DcMotor mFrontRight;
  private DcMotor mBackRight;
  private DcMotor mBackLeft;
  private ColorSensor cdsPixel;
  private int nColorSensors = 2; // IMPORTANT: Number of color sensors that you have connected
  //endregion Initial Constants

  //region Public Variables
  public String sColor = "";
  public boolean bPixelDetected = false;
  public double dDistance = 0.0;
  public float fGain = 0.0f;
  public float fHue = 0.0f;
  public float fSaturation = 0.0f;
  public float fValue = 0.0f;
  public int nColor = 0;

  //region Temporary Solution to get values from each color sensor
  public String sColorBack = "";
  public String sColorFront = "";
  public float fHueBack = 0.0f;
  public float fHueFront = 0.0f;
  public float fValueBack = 0.0f;
  public float fValueFront = 0.0f;
  public float fSaturationBack = 0.0f;
  public float fSaturationFront = 0.0f;
  public double dDistanceBack = 0.0;
  public double dDistanceFront = 0.0;
  public boolean bPixelDetectedBack = false;
  public boolean bPixelDetectedFront = false;
  int ecHomeLinear;
  int ecHighPivot;
  int ecLowPivot;
  int ecHomePivot;
  int ecLowLinear;
  int ecHighLinear;
  int ecClimbPivot;
  double mpLinear;
  double mpPivot;

  //endregion Temporary Solution to get values from each color sensor
  //endregion Public Variables

  @Override
  public void runOpMode() throws InterruptedException {
    //region Initialization

    // Declare our motors and servos
    // Make sure your ID's match your configuration
    mBackLeft = hardwareMap.get(DcMotor.class, "mBackLeft");
    mBackRight = hardwareMap.get(DcMotor.class, "mBackRight");
    mFrontLeft = hardwareMap.get(DcMotor.class, "mFrontLeft");
    mFrontRight = hardwareMap.get(DcMotor.class, "mFrontRight");
    
    ElapsedTime timer = new ElapsedTime();

    // Reverse the motors/servos as needed
    mBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    mFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    
    mBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    mBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    mFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    mFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    
    // Retrieve the IMU from the hardware map
    IMU imu = hardwareMap.get(IMU.class, "imu");
    // Adjust the orientation parameters to match your robot
    IMU.Parameters parameters = new IMU.Parameters(
      new RevHubOrientationOnRobot(
        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
      )
    );
    // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
    imu.initialize(parameters);

    fGain = 2;
    ecHomePivot = 1000;
    ecLowPivot = 3350;
    ecHighPivot = 3800;
    ecHomeLinear = 5;
    ecLowLinear = 1000;
    ecHighLinear = 2000;
    ecClimbPivot = 3500;
    mpLinear = 0.7;
    mpPivot = 0.9;

    //endregion Initialization

    telemetry.addData("Field Oriented Linear", "Press start to continue...");
    telemetry.update();
    waitForStart();

    if (isStopRequested()) return;

    while (opModeIsActive()) {
      double y = -gamepad1.left_stick_y / 1.75; // Remember, Y stick value is reversed
      double x = gamepad1.left_stick_x / 1.75;
      double rx = gamepad1.right_stick_x / 1.75;

      // Adjust the gain.
      if (gamepad1.a) {
        fGain += 0.005;
      } else if (gamepad1.b && fGain >= 1.005) {
        fGain += -0.005;
      }

      //region Gyro
      // This button choice was made so that it is hard to hit on accident,
      // it can be freely changed based on preference.
      // The equivalent button is start on Xbox-style controllers.
      if (gamepad1.back) {
        imu.resetYaw();
      }

      double botHeading = imu
        .getRobotYawPitchRollAngles()
        .getYaw(AngleUnit.RADIANS);

      // Rotate the movement direction counter to the bot's rotation
      double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
      double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

      rotX = rotX * 1.1; // Counteract imperfect strafing

      // Denominator is the largest motor power (absolute value) or 1
      // This ensures all the powers maintain the same ratio,
      // but only if at least one is out of the range [-1, 1]
      double denominator = Math.max(
        Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx),
        1
      );
      double frontLeftPower = (rotY + rotX + rx) / denominator;
      double backLeftPower = (rotY - rotX + rx) / denominator;
      double frontRightPower = (rotY - rotX - rx) / denominator;
      double backRightPower = (rotY + rotX - rx) / denominator;

      //endregion Gyro

    
      mFrontLeft.setPower(frontLeftPower);
      mBackLeft.setPower(backLeftPower);
      mFrontRight.setPower(frontRightPower);
      mBackRight.setPower(backRightPower);

      DiagnosticTelemetry();
    
    }
  }

  //region Functions

  public void DiagnosticTelemetry() {
    // String csName = "";

    // Motor powers
    telemetry.addData("FL %", mFrontLeft.getPower());
    telemetry.addData("FR %", mFrontRight.getPower());
    telemetry.addData("BL %", mBackLeft.getPower());
    telemetry.addData("BR %", mBackRight.getPower());
   
    // color

    // nColor = normalizedColors.toColor();
    // fHue = JavaUtil.colorToHue(nColor);
    // fSaturation = JavaUtil.colorToSaturation(nColor);
    // fValue = JavaUtil.colorToValue(nColor);
    //   if (fHue >= 80 && fHue <= 90) {
    //   sColor = "Yellow";
    //   bPixelDetected = true;
    // } else if (fHue >= 120 && fHue <= 134) {
    //   sColor = "Green";
    //   bPixelDetected = true;
    // } else if (fHue >= 100 && fHue <= 170) {
    //   sColor = "White";
    //   bPixelDetected = true;
    // } else if (fHue >= 100 && fHue <= 210) {
    //   sColor = "Purple";
    //   bPixelDetected = true;
    // } else {
    // sColor = "Unknown";
    //   bPixelDetected = false;
    // }

    telemetry.addData("Front Left Encoder", mFrontLeft.getCurrentPosition());
    telemetry.addData("Front Right Encoder", mFrontRight.getCurrentPosition());
    telemetry.addData("Back Left Encoder", mBackLeft.getCurrentPosition());
    telemetry.addData("Back Right Encoder", mBackRight.getCurrentPosition());
 
    telemetry.update();
  }

  //endregion Functions
}
