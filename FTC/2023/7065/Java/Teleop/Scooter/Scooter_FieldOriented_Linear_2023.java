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

@TeleOp(name = "Competition Teleop 2023", group = "Linear Opmode")
public class Scooter_FieldOriented_Linear_2023 extends LinearOpMode {

  //region Initial Constants
  private DcMotor mFrontLeft;
  private DcMotor mFrontRight;
  private DcMotor mBackRight;
  private DcMotor mBackLeft;
  private DcMotor backEncoder;
  private CRServo sBackLeftIntake;
  private CRServo sBackRightIntake;
  private CRServo sFrontLeftIntake;
  private CRServo sFrontRightIntake;
  private CRServo sDroneShooter;
  private DcMotor mLinear1;
  private DcMotor mLinear2;
  private DcMotor mPivot;

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
    backEncoder = hardwareMap.get(DcMotor.class, "backEncoder");
    mLinear1 = hardwareMap.get(DcMotor.class, "mLinear1");
    mLinear2 = hardwareMap.get(DcMotor.class, "mLinear2");
    mPivot = hardwareMap.get(DcMotor.class, "mPivot");
    sBackLeftIntake = hardwareMap.get(CRServo.class, "sBackLeftIntake");
    sBackRightIntake = hardwareMap.get(CRServo.class, "sBackRightIntake");
    sFrontLeftIntake = hardwareMap.get(CRServo.class, "sFrontLeftIntake");
    sFrontRightIntake = hardwareMap.get(CRServo.class, "sFrontRightIntake");
    sDroneShooter = hardwareMap.get(CRServo.class, "sDroneShooter");

    ElapsedTime timer = new ElapsedTime();

    mLinear1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    mLinear2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    mPivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    mLinear1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    mLinear2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    mPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    // Reverse the motors/servos as needed
    mBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    mFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    mLinear2.setDirection(DcMotor.Direction.REVERSE);
    mPivot.setDirection(DcMotor.Direction.REVERSE);
    sBackLeftIntake.setDirection(CRServo.Direction.REVERSE);
    sBackRightIntake.setDirection(CRServo.Direction.FORWARD);
    sFrontLeftIntake.setDirection(CRServo.Direction.REVERSE);
    sFrontRightIntake.setDirection(CRServo.Direction.FORWARD);
    sDroneShooter.setDirection(CRServo.Direction.REVERSE);

    mBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    mBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    mFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    mFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    mLinear1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    mLinear2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    mPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

      //region Color Sensor Function Read
      // This will look at all the configured color sensors and get a value for color as well as pixel detection
      int nColorSensor = 1;
      for (int i = 1; i <= nColorSensors; i++) {
        PixelColorSensor(i);
      }
      //endregion Color Sensor Function Read

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

      // if (gamepad1.x) {
      //   mPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      //   mPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      //   mPivot.setPower(-0.5);
      //   sleep(700);
      // } else {
      //   mPivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      //   //  mPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

      // }

      // if (gamepad1.y) {
      //   mPivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      //   mPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      //   mPivot.setPower(0.5);
      //   sleep(700);
      // } else {
      //   mPivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      //   //  mPivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

      // }
      if (gamepad1.right_bumper) {
        sDroneShooter.setPower(-1);
      } else {
        sDroneShooter.setPower(0);
      }
      if (gamepad2.left_bumper) {
        mPivot.setPower(mpPivot);
        mPivot.setTargetPosition(-5);
        mPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(1);
        mLinear1.setPower(mpLinear);
        mLinear1.setTargetPosition(Math.abs(5));
        mLinear1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mLinear2.setPower(mpLinear);
        mLinear2.setTargetPosition(5);
        mLinear2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //region Move Motors based on color sensor readout
        // If back color sensor detects a pixel, stop back motors and start detecting a pixel only for front color sensor.
        // If front color sensor and back color sensor detect a pixel stop both front and back motors
        if (bPixelDetectedBack) {
          sBackLeftIntake.setPower(0);
          sBackRightIntake.setPower(0);
          if (bPixelDetectedFront) {
            sFrontLeftIntake.setPower(0);
            sFrontRightIntake.setPower(0);
          } else {
            sFrontLeftIntake.setPower(0.5);
            sFrontRightIntake.setPower(0.5);
          }
        } else {
          sBackLeftIntake.setPower(1);
          sBackRightIntake.setPower(1);
          sFrontLeftIntake.setPower(1);
          sFrontRightIntake.setPower(1);
        }
        //endregion Move Motors based on color sensor readout
      } else {
        if (gamepad2.left_trigger > 0.17) {
          sFrontLeftIntake.setPower(1);
          sBackLeftIntake.setPower(1);
          sFrontRightIntake.setPower(1);
          sBackRightIntake.setPower(1);
        } else if (gamepad2.right_trigger > 0.17) {
          sFrontLeftIntake.setPower(-1);
          sBackLeftIntake.setPower(-1);
          sFrontRightIntake.setPower(-1);
          sBackRightIntake.setPower(-1);
        } else {
          sFrontLeftIntake.setPower(0);
          sBackLeftIntake.setPower(0);
          sFrontRightIntake.setPower(0);
          sBackRightIntake.setPower(0);
        }
      }

      if (gamepad2.right_bumper) {
        mLinear1.setPower(mpLinear);
        mLinear1.setTargetPosition(Math.abs(5));
        mLinear1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mLinear2.setPower(mpLinear);
        mLinear2.setTargetPosition(5);
        mLinear2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(500);
        mPivot.setPower(mpPivot);

        mPivot.setTargetPosition(-70);
        mPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      }
      
      
      if (gamepad2.b) {
        mPivot.setPower(mpPivot);
        mPivot.setTargetPosition(-ecLowPivot);
        mPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(1300);
        mLinear1.setPower(mpLinear);
        mLinear1.setTargetPosition(Math.abs(ecLowLinear));
        mLinear1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mLinear2.setPower(mpLinear);
        mLinear2.setTargetPosition(ecLowLinear);
        mLinear2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      }
      if (gamepad2.a) {
        mPivot.setPower(mpPivot);
        mPivot.setTargetPosition(-ecHomePivot);
        mPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(1);
        mLinear1.setPower(mpLinear);
        mLinear1.setTargetPosition(Math.abs(ecHomeLinear));
        mLinear1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mLinear2.setPower(mpLinear);
        mLinear2.setTargetPosition(ecHomeLinear);
        mLinear2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      }
      if (gamepad2.y) {
        mPivot.setPower(-mpPivot);
        mPivot.setTargetPosition(-ecHighPivot);
        sleep(2100);
        mPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mLinear1.setPower(mpLinear);
        mLinear1.setTargetPosition(Math.abs(ecHighLinear));
        mLinear1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mLinear2.setPower(mpLinear);
        mLinear2.setTargetPosition(ecHighLinear);
        mLinear2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      }
      if (gamepad2.x) {
        
        mPivot.setPower(-mpPivot);
        mPivot.setTargetPosition(-ecHighPivot);
        sleep(2100);
        mPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      }

      // if (gamepad2.x) {
      //   timer.reset();
      //   mPivot.setPower(-mpPivot);
      //   mPivot.setTargetPosition(-ecHighPivot);
      //   mPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      //   sleep(1000);
      //   mLinear1.setPower(0.6);
      //   mLinear1.setTargetPosition(Math.abs(ecHighLinear));
      //   mLinear1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      //   mLinear2.setPower(0.6);
      //   mLinear2.setTargetPosition(ecHighLinear);
      //   mLinear2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      //   sleep(4000);
      //   mLinear1.setPower(0.6);
      //   mLinear1.setTargetPosition(Math.abs(ecHomeLinear));
      //   mLinear1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      //   mLinear2.setPower(0.6);
      //   mLinear2.setTargetPosition(ecHomeLinear);
      //   mLinear2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      //   sleep(1000);
      //   mPivot.setPower(-0.5);
      //   mPivot.setTargetPosition(ecHomePivot);
      //   mPivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      // }
      //     mLinear1.setPower(-0.7);
      //     mLinear2.setPower(-0.7);
      //   } else if (gamepad2.left_stick_y < -0.17) {
      //     mLinear1.setPower(0.7);
      //     mLinear2.setPower(0.7);
      //   } else {
      //     mLinear1.setPower(0);
      //     mLinear2.setPower(0);
      //   }
      //   if (gamepad2.right_stick_y > 0.17) {
      //     mPivot.setPower(1);
      //   } else if (gamepad2.right_stick_y < -0.17) {
      //     mPivot.setPower(-1);
      //   } else {
      //     mPivot.setPower(0);
      //   }

      mFrontLeft.setPower(frontLeftPower);
      mBackLeft.setPower(backLeftPower);
      mFrontRight.setPower(frontRightPower);
      mBackRight.setPower(backRightPower);

      // if (gamepad2.a == false && gamepad2.y == false && gamepad2.b == false & gamepad2.x == false) {
      //   if (gamepad2.left_stick_y > 0.17) {
      //     mLinear1.setPower(-0.7);
      //     mLinear2.setPower(-0.7);
      //   } else if (gamepad2.left_stick_y < -0.17) {
      //     mLinear1.setPower(0.7);
      //     mLinear2.setPower(0.7);
      //   } else {
      //     mLinear1.setPower(0);
      //     mLinear2.setPower(0);
      //   }
      //   if (gamepad2.right_stick_y > 0.17) {
      //     mPivot.setPower(1);
      //   } else if (gamepad2.right_stick_y < -0.17) {
      //     mPivot.setPower(-1);
      //   } else {
      //     mPivot.setPower(0);
      //   }
      //   if (gamepad2.left_trigger > 0.17) {
      //     sFrontLeftIntake.setPower(1);
      //     sBackLeftIntake.setPower(1);
      //     sFrontRightIntake.setPower(1);
      //     sBackRightIntake.setPower(1);
      //   } else if (gamepad2.right_trigger > 0.17) {
      //     sFrontLeftIntake.setPower(-1);
      //     sBackLeftIntake.setPower(-1);
      //     sFrontRightIntake.setPower(-1);
      //     sBackRightIntake.setPower(-1);
      //   } else {
      //     sFrontLeftIntake.setPower(0);
      //     sBackLeftIntake.setPower(0);
      //     sFrontRightIntake.setPower(0);
      //     sBackRightIntake.setPower(0);
      //   }
      // }

      DiagnosticTelemetry();
      // telemetry.addData("FL %", frontLeftPower);
      // telemetry.addData("FR %", frontRightPower);
      // telemetry.addData("BL %", backLeftPower);
      // telemetry.addData("BR %", backRightPower);
      // telemetry.addData("L2 %", mLinear2.getPower());
      // telemetry.addData("L1 %", mLinear1.getPower());
      // telemetry.update();
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
    telemetry.addData("L2 %", mLinear2.getPower());
    telemetry.addData("L1 %", mLinear1.getPower());

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

    telemetry.addData("Back Color", sColorBack);
    telemetry.addData("Back Pixel Detected", bPixelDetectedBack);

    telemetry.addData("Front Color", sColorFront);
    telemetry.addData("Front Pixel Detected", bPixelDetectedFront);

    telemetry.addData(
      "Back Hue",
      Double.parseDouble(JavaUtil.formatNumber(fHueBack, 0))
    );

    telemetry.addData(
      "Front Hue",
      Double.parseDouble(JavaUtil.formatNumber(fHueFront, 0))
    );

    telemetry.addData(
      "Back Value",
      Double.parseDouble(JavaUtil.formatNumber(fValueBack, 0))
    );

    telemetry.addData(
      "Front Value",
      Double.parseDouble(JavaUtil.formatNumber(fValueFront, 0))
    );

    telemetry.addData(
      "Back Saturation",
      Double.parseDouble(JavaUtil.formatNumber(fSaturationBack, 0))
    );

    telemetry.addData(
      "Front Saturation",
      Double.parseDouble(JavaUtil.formatNumber(fSaturationFront, 0))
    );

    telemetry.addData(
      "Dist to tgt (cm)",
      ((DistanceSensor) cdsPixel).getDistance(DistanceUnit.CM)
    );

    telemetry.addData("Front Left Encoder", mFrontLeft.getCurrentPosition());
    telemetry.addData("Front Right Encoder", mFrontRight.getCurrentPosition());
    telemetry.addData("Back Left Encoder", mBackLeft.getCurrentPosition());
    telemetry.addData("Back Right Encoder", mBackRight.getCurrentPosition());
    telemetry.addData("Back Dead Wheel", backEncoder.getCurrentPosition());
    telemetry.addData("Linear 1 Encoder", mLinear1.getCurrentPosition());
    telemetry.addData("Linear 2 Encoder", mLinear2.getCurrentPosition());
    telemetry.addData("Pivot Encoder", mPivot.getCurrentPosition());

    // telemetry.addData(
    //   "Light detected",
    //   (
    //     (OpticalDistanceSensor) cdsPixel
    //   ).getLightDetected()
    // );

    // telemetry.addData(
    //   "Gain",
    //   ((NormalizedColorSensor) cdsPixel).getGain()
    // );

    // telemetry.addData(
    //   "Saturation",
    //   Double.parseDouble(JavaUtil.formatNumber(fSaturation, 3))
    // );

    telemetry.update();
  }

  public void PixelColorSensor(int nColorSensor) {
    NormalizedRGBA normalizedColors;
    // String csName = "";

    switch (nColorSensor) {
      case 1:
        // Use Back Color Sensor
        cdsPixel = hardwareMap.get(ColorSensor.class, "csBackIntake");
        // csName = "Back Color Sensor";
        break;
      case 2:
        // Use Front Color Sensor
        cdsPixel = hardwareMap.get(ColorSensor.class, "csFrontIntake");
        // csName = "Front Color Sensor";
        break;
      default:
        // Use Back Color Sensor
        cdsPixel = hardwareMap.get(ColorSensor.class, "csBackIntake");
        // csName = "Back Color Sensor";
        break;
    }

    JavaUtil.showColor(hardwareMap.appContext, nColor);
    // Display distance info.
    // telemetry.addData(
    //   "Dist to tgt (cm)",
    //   ((DistanceSensor) cdsPixel).getDistance(
    //       DistanceUnit.CM
    //     )
    // );
    // Display reflected light.
    // telemetry.addData(
    //   "Light detected",
    //   (
    //     (OpticalDistanceSensor) cdsPixel
    //   ).getLightDetected()
    // );

    ((NormalizedColorSensor) cdsPixel).setGain(fGain);
    // telemetry.addData(
    //   "Gain",
    //   ((NormalizedColorSensor) cdsPixel).getGain()
    // );
    // Read color from the sensor.
    normalizedColors = ((NormalizedColorSensor) cdsPixel).getNormalizedColors();

    nColor = normalizedColors.toColor();
    fHue = JavaUtil.colorToHue(nColor);
    fSaturation = JavaUtil.colorToSaturation(nColor);
    fValue = JavaUtil.colorToValue(nColor);
    // telemetry.addData(
    //   csName + " " + "Hue",
    //   Double.parseDouble(JavaUtil.formatNumber(fHue, 0))
    // );
    // telemetry.addData(
    //   "Saturation",
    //   Double.parseDouble(JavaUtil.formatNumber(fSaturation, 3))
    // );
    // Show the color on the Robot Controller screen.
    // Use hue to determine if it's red, green, blue, etc..
    // if (fHue == 180) {
    //   sColor = "none";
    //   bPixelDetected = false;
    // }

    // if (fHue >= 80 && fHue <= 100) {
    //   dDistance = ((DistanceSensor) cdsPixel).getDistance(DistanceUnit.CM);
    //   sColor = "Yellow";
    //   bPixelDetected = true;
    // } else if (fHue >= 120 && fHue <= 140) {
    //   dDistance = ((DistanceSensor) cdsPixel).getDistance(DistanceUnit.CM);
    //   sColor = "Green";
    //   bPixelDetected = true;
    // } else if (fHue >= 150 && fHue <= 190) {
    //   dDistance = ((DistanceSensor) cdsPixel).getDistance(DistanceUnit.CM);
    //   sColor = "White";
    //   bPixelDetected = true;
    // } else if (fHue >= 200 && fHue <= 230) {
    //   dDistance = ((DistanceSensor) cdsPixel).getDistance(DistanceUnit.CM);
    //   sColor = "Purple";
    //   bPixelDetected = true;
    if (
      ((DistanceSensor) cdsPixel).getDistance(DistanceUnit.CM) >= 0.600 &&
      ((DistanceSensor) cdsPixel).getDistance(DistanceUnit.CM) <= 1
    ) {
      bPixelDetected = true;
      dDistance = ((DistanceSensor) cdsPixel).getDistance(DistanceUnit.CM);
    } else {
      bPixelDetected = false;
    }

    // sColor = "Unknown";
    // bPixelDetected = true;
    // } else {
    //   // telemetry.addData(csName + " " + "Color", "Unknown");
    //   bPixelDetected = false;
    // }
    // Check to see if it might be black or white.
    // if (fSaturation < 0.2) {
    //   // telemetry.addData(csName + " " + "Check Sat", "Is surface white?");
    //   bPixelDetected = false;
    // }
    // telemetry.update();
    // if (fValue < 0.16) {
    //   // telemetry.addData(csName + " " + "Check Val", "Is surface black?");
    //   bPixelDetected = false;
    // }

    if (nColorSensor == 1) {
      // Use Back Color Sensor
      fHueBack = fHue;
      fValueBack = fValue;
      fSaturationBack = fSaturation;
      sColorBack = sColor;
      dDistanceBack = dDistance;
      bPixelDetectedBack = bPixelDetected;
    } else if (nColorSensor == 2) {
      // Use Front Color Sensor
      fHueFront = fHue;
      fValueFront = fValue;
      fSaturationFront = fSaturation;
      sColorFront = sColor;
      dDistanceFront = dDistance;
      bPixelDetectedFront = bPixelDetected;
    } else {
      fHueFront = 0.0f;
      fHueBack = 0.0f;
      bPixelDetectedBack = false;
      bPixelDetectedFront = false;
    }
    // telemetry.addData(csName + " " + "Color", sColor);
    // telemetry.addData(csName + " " + "Pixel Detected", bPixelDetected);
    // telemetry.update();
  }
  //endregion Functions
}
