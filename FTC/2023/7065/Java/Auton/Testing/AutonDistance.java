package org.firstinspires.ftc.teamcode.Auton;

import android.graphics.Color;
//region imports
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//endregion imports
@Disabled
@Autonomous(
  name = "AutonDistance",
  group = "Blue",
  preselectTeleOp = "Scooter Drivetrain 2023"
)
public class AutonDistance extends LinearOpMode {

  //region Declaring Constants/Variables
  private DcMotor mBackLeft = null;
  private DcMotor mBackRight = null;
  private DcMotor mFrontLeft = null;
  private DcMotor mFrontRight = null;
  // private DcMotor mLights = null;
  private double driveSpeed = 0;
  private double headingError = 0;
  private double headingOffset = 0;
  private double leftSpeed = 0;
  private double rightSpeed = 0;
  private double robotHeading = 0;
  private double targetHeading = 0;
  private double turnSpeed = 0;
  private int leftTarget = 0;
  private int rightTarget = 0;
  public String sColor = "";
  public boolean bTeamPropDetected = false;
  public double dDistance = 0.0;
  public float fGain = 0.0f;
  public float fHue = 0.0f;
  public float fSaturation = 0.0f;
  public float fValue = 0.0f;
  public int nColor = 0;
  public String sColorChassis = "";
  public float fHueChassis = 0.0f;
  public float fValueChassis = 0.0f;
  public float fSaturationChassis = 0.0f;
  public double dDistanceChassis = 0.0;
  public boolean bTeamPropDetectedChassis = false;
  private int nSquareSize = 24;

  static final double COUNTS_PER_MOTOR_REV = 637.304; // eg: GoBILDA 312 RPM Yellow Jacket
  static final double DRIVE_GEAR_REDUCTION = 1.0; // No External Gearing.
  static final double DRIVE_SPEED = 0.8; // Max driving speed for better distance accuracy.
  static final double HEADING_THRESHOLD = 1.0; // How close must the heading get to the target before moving to next step.
  static final double P_DRIVE_GAIN = 0.03; // Larger is more responsive, but also less stable
  static final double P_TURN_GAIN = 0.04; // Larger is more responsive, but also less stable
  static final double TURN_SPEED = 0.8; // Max Turn speed to limit turn rate
  static final double WHEEL_DIAMETER_INCHES = 3.77953; // For figuring circumference
  static final double COUNTS_PER_INCH =
    (
      (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
      (WHEEL_DIAMETER_INCHES * 3.1415)
    ); // Inch to encoder count
  private BNO055IMU imu = null; // Control/Expansion Hub IMU

  //endregion Declaring Constants/Variables

  @Override
  public void runOpMode() {
    //region Initialization
    mBackRight = hardwareMap.get(DcMotor.class, "mBackRight");
    mBackLeft = hardwareMap.get(DcMotor.class, "mBackLeft");
    mFrontRight = hardwareMap.get(DcMotor.class, "mFrontRight");
    mFrontLeft = hardwareMap.get(DcMotor.class, "mFrontLeft");
    // mLights = hardwareMap.get(DcMotor.class, "mLights");

    // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
    // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
    // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
    mFrontLeft.setDirection(DcMotor.Direction.REVERSE);
    mBackLeft.setDirection(DcMotor.Direction.REVERSE);

    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    imu = hardwareMap.get(BNO055IMU.class, "imu");
    imu.initialize(parameters);

    // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
    mBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    mBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    mFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    mFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    mFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    mFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    mBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    mBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    fGain = 2;

    // // Wait for the game to start (Display Gyro value while waiting)
    while (opModeInInit()) {
      telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
      telemetry.update();
      // mLights.setPower(1);
      mBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      mBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      mFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      mFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    telemetry.addData("Action needed", "Please press the start triangle");
    telemetry.update();
    //endregion Initialization

    // Wait for start command from Driver Station.

    waitForStart();
    if (opModeIsActive()) {
      int nColorSensor = 1;
      for (int i = 1; i <= nColorSensors; i++) {
        PixelColorSensor(i);
      }
      DriveRobotEncoder(1, 0.3, 4, 2); // Strafe left 2 inches
      driveStraight(0.3, 12, 0); // Drive forward 12 inches
      turnToHeading(0.3, 90); // Turn foward 90 degrees
      DriveRobotEncoder(1, 0.3, 4, 5); // Strafe left 5 inches
      driveStraight(0.3, 4, 0); // Drive forward 4 inches
      DriveRobotEncoder(1, 0.3, 5, 5); // Strafe right 5 inches
      driveStraight(0.3, 10, 0); // Drive forward 10 inches

      if (bTeamPropDetectedChassis) {
        sBackLeftIntake.setPower(-1);
        sBackRightIntake.setPower(-1);
        sFrontLeftIntake.setPower(-1);
        sFrontRightIntake.setPower(-1);
      }

      // driveStraight(0.3, 10, 0);
      // sleep(500);
      // turnToHeading(0.3, 90);
      // sleep(500);
      // DriveRobotEncoder(1, 0.5, 5, 5);
      // DriveRobotEncoder(1, 0.5, 5, (nSquareSize + 3)); // Strafe right 27 inches from wall
      // resetHeading();

      // driveStraight(DRIVE_SPEED, (nSquareSize * 2.5), 0); // Drive straight for 26.5 inches

      // DriveRobotEncoder(1, 0.5, 4, (nSquareSize - 2.5)); // Strafe left 24 inches
      // resetHeading();

      // driveStraight(DRIVE_SPEED, (nSquareSize - 6), 0); // Drive straight 19 inches to get to the back stage

      // PixelColorSensor(1);

      mFrontLeft.setPower(0);
      mFrontRight.setPower(0);
      mBackLeft.setPower(0);
      mBackRight.setPower(0);
    }
  }

  //region Functions

  public void driveStraight(
    double maxDriveSpeed,
    double distance,
    double heading
  ) {
    mFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    mBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    mFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    mBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    // Ensure that the opmode is still active
    if (opModeIsActive()) {
      // Determine new target position, and pass to motor controller
      int moveCounts = (int) (distance * COUNTS_PER_INCH);
      leftTarget = mFrontLeft.getCurrentPosition() + moveCounts;
      leftTarget = mBackLeft.getCurrentPosition() + moveCounts;
      rightTarget = mFrontRight.getCurrentPosition() + moveCounts;
      rightTarget = mBackRight.getCurrentPosition() + moveCounts;

      // Set Target FIRST, then turn on RUN_TO_POSITION
      mFrontLeft.setTargetPosition(leftTarget);
      mBackRight.setTargetPosition(rightTarget);
      mBackLeft.setTargetPosition(leftTarget);
      mFrontRight.setTargetPosition(rightTarget);

      mFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      mFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      mBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      mBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

      // Set the required driving speed  (must be positive for RUN_TO_POSITION)
      // Start driving straight, and then enter the control loop
      maxDriveSpeed = Math.abs(maxDriveSpeed);
      moveRobot(maxDriveSpeed, 0);

      // keep looping while we are still active, and BOTH motors are running.

      while (
        mFrontLeft.isBusy() &&
        mFrontRight.isBusy() &&
        mBackLeft.isBusy() &&
        mBackRight.isBusy()
      ) {
        // Determine required steering to keep on heading
        turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

        // if driving in reverse, the motor correction also needs to be reversed
        if (distance < 0) turnSpeed *= -1.0;

        // Apply the turning correction to the current driving speed.
        moveRobot(driveSpeed, turnSpeed);

        // Display drive status for the driver.
        sendTelemetry(true);
      }

      // // Stop all motion & Turn off RUN_TO_POSITION
      moveRobot(0, 0);
      mFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      mFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      mBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      mBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
  }

  public void turnToHeading(double maxTurnSpeed, double heading) {
    // Run getSteeringCorrection() once to pre-calculate the current error
    // mFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    // mBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    // mFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    // mBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    getSteeringCorrection(heading, P_DRIVE_GAIN);

    // keep looping while we are still active, and not on heading.
    while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {
      // Determine required steering to keep on heading
      turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

      // Clip the speed to the maximum permitted value.
      turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

      // Pivot in place by applying the turning correction
      moveRobot(0, turnSpeed);

      // Display drive status for the driver.
      sendTelemetry(false);
    }

    // Stop all motion;
    moveRobot(0, 0);
  }

  public void DriveRobot(int ms, double mp, int nDirection) {
    switch (nDirection) {
      case 0:
        // Drive Forward
        mFrontLeft.setPower(mp);
        mBackLeft.setPower(mp);
        mFrontRight.setPower(mp);
        mBackRight.setPower(mp);
        break;
      case 1:
        // Drive Reverse
        mFrontLeft.setPower(-mp);
        mBackLeft.setPower(-mp);
        mFrontRight.setPower(-mp);
        mBackRight.setPower(-mp);
        break;
      case 2:
        // Turn Left
        mFrontLeft.setPower(-mp);
        mBackLeft.setPower(-mp);
        mFrontRight.setPower(mp);
        mBackRight.setPower(mp);
        break;
      case 3:
        // Turn Right
        mFrontLeft.setPower(mp);
        mBackLeft.setPower(mp);
        mFrontRight.setPower(-mp);
        mBackRight.setPower(-mp);
        break;
      case 4:
        // Strafe Left
        mFrontLeft.setPower(-mp);
        mBackLeft.setPower(mp);
        mFrontRight.setPower(mp);
        mBackRight.setPower(-mp);
        break;
      case 5:
        // Strafe Right
        mFrontLeft.setPower(mp);
        mBackLeft.setPower(-mp);
        mFrontRight.setPower(-mp);
        mBackRight.setPower(mp);
        break;
      default:
        mFrontLeft.setPower(0);
        mBackLeft.setPower(0);
        mFrontRight.setPower(0);
        mBackRight.setPower(0);
        break;
    }
    sleep(ms);
  }

  public void DriveRobotEncoder(
    int ms,
    double mp,
    int nDirection,
    double dInch
  ) {
    int Ec = (int) COUNTS_PER_INCH * (int) dInch;

    mFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    mBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    mFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    mBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    switch (nDirection) {
      case 0:
        // Drive Forward

        // Encoder Count Drive Forward - Start
        mFrontLeft.setPower(mp);
        mBackLeft.setPower(mp);
        mFrontRight.setPower(mp);
        mBackRight.setPower(mp);
        mFrontLeft.setTargetPosition(Ec);
        mBackLeft.setTargetPosition(Ec);
        mFrontRight.setTargetPosition(Ec);
        mBackRight.setTargetPosition(Ec);
        mFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // mFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // mBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // mFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // mBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (
          !(
            !mFrontLeft.isBusy() &&
            !mBackLeft.isBusy() &&
            !mFrontRight.isBusy() &&
            !mBackRight.isBusy()
          )
        ) {}
        // Encoder Count Drive Forward - End

        break;
      case 1:
        // Drive Reverse
        mFrontLeft.setPower(mp);
        mBackLeft.setPower(mp);
        mFrontRight.setPower(mp);
        mBackRight.setPower(mp);
        mFrontLeft.setTargetPosition(-Ec);
        mBackLeft.setTargetPosition(-Ec);
        mFrontRight.setTargetPosition(-Ec);
        mBackRight.setTargetPosition(-Ec);
        mFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // mFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // mBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // mFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // mBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (
          !(
            !mFrontLeft.isBusy() &&
            !mBackLeft.isBusy() &&
            !mFrontRight.isBusy() &&
            !mBackRight.isBusy()
          )
        ) {}
        break;
      case 2:
        // Turn Left
        mFrontLeft.setPower(mp);
        mBackLeft.setPower(mp);
        mFrontRight.setPower(mp);
        mBackRight.setPower(mp);
        mFrontLeft.setTargetPosition(Ec);
        mBackLeft.setTargetPosition(Ec);
        mFrontRight.setTargetPosition(-Ec);
        mBackRight.setTargetPosition(-Ec);
        mFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // mFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // mBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // mFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // mBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (
          !(
            !mFrontLeft.isBusy() &&
            !mBackLeft.isBusy() &&
            !mFrontRight.isBusy() &&
            !mBackRight.isBusy()
          )
        ) {}
        break;
      case 3:
        // Turn Right
        mFrontLeft.setPower(mp);
        mBackLeft.setPower(mp);
        mFrontRight.setPower(mp);
        mBackRight.setPower(mp);
        mFrontLeft.setTargetPosition(-Ec);
        mBackLeft.setTargetPosition(-Ec);
        mFrontRight.setTargetPosition(Ec);
        mBackRight.setTargetPosition(Ec);
        mFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // mFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // mBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // mFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // mBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (
          !(
            !mFrontLeft.isBusy() &&
            !mBackLeft.isBusy() &&
            !mFrontRight.isBusy() &&
            !mBackRight.isBusy()
          )
        ) {}
        break;
      case 4:
        // Strafe Left
        mFrontLeft.setPower(mp);
        mBackLeft.setPower(mp);
        mFrontRight.setPower(mp);
        mBackRight.setPower(mp);
        mFrontLeft.setTargetPosition(-Ec);
        mBackLeft.setTargetPosition(Ec);
        mFrontRight.setTargetPosition(Ec);
        mBackRight.setTargetPosition(-Ec);
        mFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // mFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // mBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // mFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // mBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (
          !(
            !mFrontLeft.isBusy() &&
            !mBackLeft.isBusy() &&
            !mFrontRight.isBusy() &&
            !mBackRight.isBusy()
          )
        ) {}
        break;
      case 5:
        // Strafe Right
        mFrontLeft.setPower(mp);
        mBackLeft.setPower(mp);
        mFrontRight.setPower(mp);
        mBackRight.setPower(mp);
        mFrontLeft.setTargetPosition(Ec);
        mBackLeft.setTargetPosition(-Ec);
        mFrontRight.setTargetPosition(-Ec);
        mBackRight.setTargetPosition(Ec);
        mFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // mFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // mBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // mFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // mBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (
          !(
            !mFrontLeft.isBusy() &&
            !mBackLeft.isBusy() &&
            !mFrontRight.isBusy() &&
            !mBackRight.isBusy()
          )
        ) {}

        break;
      default:
        mFrontLeft.setPower(0);
        mBackLeft.setPower(0);
        mFrontRight.setPower(0);
        mBackRight.setPower(0);
        break;
    }

    sleep(ms);
    telemetry.addData("Ec", Ec);
    telemetry.update();
  }

  public void holdHeading(
    double maxTurnSpeed,
    double heading,
    double holdTime
  ) {
    ElapsedTime holdTimer = new ElapsedTime();
    holdTimer.reset();

    // keep looping while we have time remaining.
    while (opModeIsActive() && (holdTimer.time() < holdTime)) {
      // Determine required steering to keep on heading
      turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

      // Clip the speed to the maximum permitted value.
      turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

      // Pivot in place by applying the turning correction
      moveRobot(0, turnSpeed);

      // Display drive status for the driver.
      sendTelemetry(false);
    }

    // Stop all motion;
    moveRobot(0, 0);
  }

  public double getSteeringCorrection(
    double desiredHeading,
    double proportionalGain
  ) {
    targetHeading = desiredHeading; // Save for telemetry

    // Get the robot heading by applying an offset to the IMU heading
    robotHeading = getRawHeading() - headingOffset;

    // Determine the heading current error
    headingError = targetHeading - robotHeading;

    // Normalize the error to be within +/- 180 degrees
    while (headingError > 180) headingError -= 360;
    while (headingError <= -180) headingError += 360;

    // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
    return Range.clip(headingError * proportionalGain, -1, 1);
  }

  public void moveRobot(double drive, double turn) {
    driveSpeed = drive; // save this value as a class member so it can be used by telemetry.
    turnSpeed = turn; // save this value as a class member so it can be used by telemetry.

    leftSpeed = drive - turn;
    rightSpeed = drive + turn;

    // Scale speeds down if either one exceeds +/- 1.0;
    double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
    if (max > 1.0) {
      leftSpeed /= max;
      rightSpeed /= max;
    }

    mFrontLeft.setPower(leftSpeed);
    mBackLeft.setPower(leftSpeed);
    mBackRight.setPower(rightSpeed);
    mFrontRight.setPower(rightSpeed);
  }

  private void sendTelemetry(boolean straight) {
    if (straight) {
      telemetry.addData("Motion", "Drive Straight");
      telemetry.addData("Target Pos L:R", "%7d:%7d", leftTarget, rightTarget);
      telemetry.addData(
        "Actual Pos L:R",
        "%7d:%7d",
        mFrontLeft.getCurrentPosition(),
        mFrontRight.getCurrentPosition(),
        mBackRight.getCurrentPosition()
      );
      mBackLeft.getCurrentPosition();
    } else {
      telemetry.addData("Motion", "Turning");
    }

    telemetry.addData(
      "Angle Target:Current",
      "%5.2f:%5.0f",
      targetHeading,
      robotHeading
    );
    telemetry.addData("Error:Steer", "%5.1f:%5.1f", headingError, turnSpeed);
    telemetry.addData(
      "Wheel Speeds L:R.",
      "%5.2f : %5.2f",
      leftSpeed,
      rightSpeed
    );
    telemetry.update();
  }

  public double getRawHeading() {
    Orientation angles = imu.getAngularOrientation(
      AxesReference.INTRINSIC,
      AxesOrder.ZYX,
      AngleUnit.DEGREES
    );
    return angles.firstAngle;
  }

  public void resetHeading() {
    // Save a new heading offset equal to the current raw heading.
    headingOffset = getRawHeading();
    robotHeading = 0;
  }

  public void PixelColorSensor(int nColorSensor) {
    NormalizedRGBA normalizedColors;
    String csName = "";

    switch (nColorSensor) {
      case 1:
        // Use ChassisColor Sensor
        cdsPixel = hardwareMap.get(ColorSensor.class, "csChassis");
        csName = "Chassis Color";
        break;
      case 2:
        // Use Front Color Sensor
        cdsPixel = hardwareMap.get(ColorSensor.class, "csFront");
        csName = "Front Color";
        break;
      default:
        // Use Back Color Sensor
        cdsPixel = hardwareMap.get(ColorSensor.class, "csBack");
        csName = "Back Color";
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
    //   "Hue",
    //   Double.parseDouble(JavaUtil.formatNumber(fHue, 0))
    // );
    // telemetry.addData(
    //   "Saturation",
    //   Double.parseDouble(JavaUtil.formatNumber(fSaturation, 3))
    // );
    // Show the color on the Robot Controller screen.
    // Use hue to determine if it's red, green, blue, etc..

    if (
      ((DistanceSensor) cdsPixel).getDistance(DistanceUnit.CM) >= 0.600 &&
      ((DistanceSensor) cdsPixel).getDistance(DistanceUnit.CM) <= 1
    ) {
      bTeamPropDetected = true;
      dDistance = ((DistanceSensor) cdsPixel).getDistance(DistanceUnit.CM);
    } else {
      bTeamPropDetected = false;
    }
    // Check to see if it might be black or white.
    // if (fSaturation < 0.2) {
    // telemetry.addData("Check Sat", "Is surface white?");
    // }
    //telemetry.update();
    // if (fValue < 0.16) {
    //telemetry.addData("Check Val", "Is surface black?");
    // }

    if (nColorSensor == 1) {
      // Use Chassis Color Sensor
      fHueChassis = fHue;
      fValueChassis = fValue;
      fSaturationChassis = fSaturation;
      sColorChassis = sColor;
      dDistanceChassis = dDistance;
      bTeamPropDetectedChassis = bTeamPropDetected;
    } else if (nColorSensor == 2) {
      // Use Front Color Sensor
      //  fHueFront = fHue;
      //  fValueFront = fValue;
      // fSaturationFront = fSaturation;
      // sColorFront = sColor;
      //  dDistanceFront = dDistance;
      //  bPixelDetectedFront = bPixelDetected;
    } else {
      fHueFront = 0.0f;
      fHueBack = 0.0f;
      bPixelDetectedBack = false;
      bPixelDetectedFront = false;
    }

    telemetry.addData(csName, sColor);
    telemetry.addData("Team Prop Detected", bTeamPropDetected);
    telemetry.update();
  }
  //endregion Functions
}
