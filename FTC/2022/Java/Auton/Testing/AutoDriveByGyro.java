package org.firstinspires.ftc.teamcode.Competition.Auton;

//region imports
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.text.SimpleDateFormat;
import java.util.Date;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//endregion imports
//@Disabled
@Autonomous(
  name = "Auto Drive By Gyro",
  group = "Competition",
  preselectTeleOp = "2022 Iterative OpMode"
)
public class AutoDriveByGyro extends LinearOpMode {

  //region Declaring Constants/Variables
  private RevBlinkinLedDriver blinkin;
  /* Declare OpMode members. */
  private DcMotor mBackLeft = null;
  private DcMotor mBackRight = null;
  private DcMotor mFrontLeft = null;
  private DcMotor mFrontRight = null;
  private BNO055IMU imu = null; // Control/Expansion Hub IMU
  private DcMotor mArm = null;
  private CRServo sGrabOne = null;
  private CRServo sGrabTwo = null;
  private CRServo sGrabThree = null;

  private double robotHeading = 0;
  private double headingOffset = 0;
  private double headingError = 0;

  // These variable are declared here (as class members) so they can be updated in various methods,
  // but still be displayed by sendTelemetry()
  private double targetHeading = 0;
  private double driveSpeed = 0;
  private double turnSpeed = 0;
  private double leftSpeed = 0;
  private double rightSpeed = 0;
  private int leftTarget = 0;
  private int rightTarget = 0;
  int CurrentColor;
  boolean Green;
  boolean Red;
  boolean Purple;

  // Calculate the COUNTS_PER_INCH for your specific drive train.
  // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
  // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
  // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
  // This is gearing DOWN for less speed and more torque.
  // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
  static final double COUNTS_PER_MOTOR_REV = 637.304; // eg: GoBILDA 312 RPM Yellow Jacket
  static final double DRIVE_GEAR_REDUCTION = 1.0; // No External Gearing.
  static final double WHEEL_DIAMETER_INCHES = 3.77953; // For figuring circumference
  static final double COUNTS_PER_INCH =
    (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
    (WHEEL_DIAMETER_INCHES * 3.1415);

  // These constants define the desired driving/control characteristics
  // They can/should be tweaked to suit the specific robot drive train.
  static final double DRIVE_SPEED = 0.8; // Max driving speed for better distance accuracy.
  static final double TURN_SPEED = 0.8; // Max Turn speed to limit turn rate
  static final double HEADING_THRESHOLD = 1.0; // How close must the heading get to the target before moving to next step.
  // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
  // Define the Proportional control coefficient (or GAIN) for "heading control".
  // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
  // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
  // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
  static final double P_TURN_GAIN = 0.02; // Larger is more responsive, but also less stable
  static final double P_DRIVE_GAIN = 0.03; // Larger is more responsive, but also less stable

  //endregion Declaring Constants/Variables

  //region Functions
  /*
   * ====================================================================================================
   * Driving "Helper" functions are below this line.
   * These provide the high and low level methods that handle driving straight and turning.
   * ====================================================================================================
   */

  // **********  HIGH Level driving functions.  ********************

  /**
   *  Method to drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
   *  Move will stop if either of these conditions occur:
   *  1) Move gets to the desired position
   *  2) Driver stops the opmode running.
   *
   * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
   * @param distance   Distance (in inches) to move from current position.  Negative distance means move backward.
   * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
   *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
   *                   If a relative angle is required, add/subtract from the current robotHeading.
   */

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

      // Stop all motion & Turn off RUN_TO_POSITION
      moveRobot(0, 0);
      mFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      mFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      mBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      mBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
  }

  /**
   *  Method to spin on central axis to point in a new direction.
   *  Move will stop if either of these conditions occur:
   *  1) Move gets to the heading (angle)
   *  2) Driver stops the opmode running.
   *
   * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
   * @param heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
   *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
   *              If a relative angle is required, add/subtract from current heading.
   */

  public void turnToHeading(double maxTurnSpeed, double heading) {
    // Run getSteeringCorrection() once to pre-calculate the current error
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

  // region functions

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
    // int ecFR;
    // double inFR;
    // int ecPerInch;

    // ecFR = 637.304;
    // inFR = 12.75;
    // ecPerInch = ecFR / (int) inFR;

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

  public void ArmEncoder(double mp, int eC, int ms) {
    mArm.setPower(mp);
    mArm.setTargetPosition(eC);
    mArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    while (!(!mArm.isBusy())) {}
    sleep(ms);
  }

  // public void Servo(double sp, int CurrentColor) {
  //   CurrentColor = Color.rgb(Color_Sensor_REV_ColorRangeSensor.red(), Color_Sensor_REV_ColorRangeSensor.green(), Color_Sensor_REV_ColorRangeSensor.blue());

  // }
  // endregion functions

  /**
   *  Method to obtain & hold a heading for a finite amount of time
   *  Move will stop once the requested time has elapsed
   *  This function is useful for giving the robot a moment to stabilize it's heading between movements.
   *
   * @param maxTurnSpeed      Maximum differential turn speed (range 0 to +1.0)
   * @param heading    Absolute Heading Angle (in Degrees) relative to last gyro reset.
   *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
   *                   If a relative angle is required, add/subtract from current heading.
   * @param holdTime   Length of time (in seconds) to hold the specified heading.
   */
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

  // **********  LOW Level driving functions.  ********************

  /**
   * This method uses a Proportional Controller to determine how much steering correction is required.
   *
   * @param desiredHeading        The desired absolute heading (relative to last heading reset)
   * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
   * @return                      Turning power needed to get to required heading.
   */
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

  /**
   * This method takes separate drive (fwd/rev) and turn (right/left) requests,
   * combines them, and applies the appropriate speed commands to the left and right wheel motors.
   * @param drive forward motor speed
   * @param turn  clockwise turning motor speed.
   */
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

  /**
   *  Display the various control parameters while driving
   *
   * @param straight  Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
   */
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

  /**
   * read the raw (un-offset Gyro heading) directly from the IMU
   */
  public double getRawHeading() {
    Orientation angles = imu.getAngularOrientation(
      AxesReference.INTRINSIC,
      AxesOrder.ZYX,
      AngleUnit.DEGREES
    );
    return angles.firstAngle;
  }

  /**
   * Reset the "offset" heading back to zero
   */
  public void resetHeading() {
    // Save a new heading offset equal to the current raw heading.
    headingOffset = getRawHeading();
    robotHeading = 0;
  }

  //endregion Functions

  @Override
  public void runOpMode() {
    // Initialize the drive system variables.
    blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

    mBackRight = hardwareMap.get(DcMotor.class, "mBackRight");
    mBackLeft = hardwareMap.get(DcMotor.class, "mBackLeft");
    mFrontRight = hardwareMap.get(DcMotor.class, "mFrontRight");
    mFrontLeft = hardwareMap.get(DcMotor.class, "mFrontLeft");
    mArm = hardwareMap.get(DcMotor.class, "mArm");
    sGrabOne = hardwareMap.get(CRServo.class, "sGrabOne");
    sGrabTwo = hardwareMap.get(CRServo.class, "sGrabTwo");
    sGrabThree = hardwareMap.get(CRServo.class, "sGrabThree");
    // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
    // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
    // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
    mFrontLeft.setDirection(DcMotor.Direction.REVERSE);
    mBackLeft.setDirection(DcMotor.Direction.REVERSE);
    mArm.setDirection(DcMotor.Direction.REVERSE);

    // define initialization values for IMU, and then initialize it.
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    imu = hardwareMap.get(BNO055IMU.class, "imu");
    imu.initialize(parameters);

    // CurrentColor = (Color.rgb(Color_Sensor_REV_ColorRangeSensor.red(), Color_Sensor_REV_ColorRangeSensor.green(), Color_Sensor_REV_ColorRangeSensor.blue()));
    // Green = (JavaUtil.colorToSaturation(CurrentColor) >= 0.6 && JavaUtil.colorToHue(CurrentColor) > 81 && JavaUtil.colorToHue(CurrentColor)) < 140;
    // Blue = (JavaUtil.colorToSaturation(CurrentColor) >= 0.6 && JavaUtil.colorToHue(CurrentColor) > 210 && JavaUtil.colorToHue(CurrentColor)) < 275;
    // Purple = (JavaUtil.colorToSaturation(CurrentColor) >= 0.5 && JavaUtil.colorToHue(CurrentColor) = 304);

    // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
    mBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    mBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    mFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    mFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    mFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    mFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    mBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    mBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    // Wait for the game to start (Display Gyro value while waiting)
    while (opModeInInit()) {
      telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
      telemetry.update();
    }
    telemetry.addData("Action needed", "Please press the start triangle");
    telemetry.update();
    waitForStart();
    if (opModeIsActive()) {
      // Set the encoders for closed loop speed control, and reset the heading.
      mBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      mBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      mFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      mFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      resetHeading();
      // CurrentColor = Color.rgb(Color_Sensor_REV_ColorRangeSensor.red(), Color_Sensor_REV_ColorRangeSensor.green(), Color_Sensor_REV_ColorRangeSensor.blue());
      // Green = (JavaUtil.colorToSaturation(CurrentColor) >= 0.6 && JavaUtil.colorToHue(CurrentColor) > 81 && JavaUtil.colorToHue(CurrentColor)) < 140;
      // Blue = (JavaUtil.colorToSaturation(CurrentColor) >= 0.6 && JavaUtil.colorToHue(CurrentColor) > 210 && JavaUtil.colorToHue(CurrentColor)) < 275;
      // Purple = (JavaUtil.colorToSaturation(CurrentColor) >= 0.5 && JavaUtil.colorToHue(CurrentColor) = 304);

      // Step through each leg of the path,
      // Notes:   Reverse movement is obtained by setting a negative distance (not speed)
      //          holdHeading() is used after turns to let the heading stabilize
      //          Add a sleep(2000) after any step to keep the telemetry data visible for review

      //   blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_RED);
      //   sleep(1000);
      //   telemetry.update();
      //   blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
      //   sleep(1000);
      //   telemetry.update();
      // ArmEncoder(0.7, 200, 500);
      // driveStraight(DRIVE_SPEED, 0.5, 0.0);

      // // Strafe Right
      DriveRobotEncoder(1, 0.7, 4, 20);

      // // driveStraight(DRIVE_SPEED, 5.0, 0.0);
      // // driveStraight(DRIVE_SPEED, -1.0, 0.0);
      // // turnToHeading(TURN_SPEED, -90.0);
      // // resetHeading();
      // // telemetry.update();
      // // driveStraight(DRIVE_SPEED, 18.0, 0.0);
      // // resetHeading();
      // // telemetry.update();
      // // turnToHeading(TURN_SPEED, 90.0);
      // resetHeading();
      // telemetry.update();

      // // Drive Straight 41 inches with gyro
      // driveStraight(DRIVE_SPEED, 41.0, 0.0);

      // resetHeading();
      // telemetry.update();
      // DriveRobotEncoder(1, 0.7, 4, 12);
      // ArmEncoder(0.7, 3000, 500);
      // sGrabOne.setPower(-1);
      // sGrabTwo.setPower(1);
      // DriveRobotEncoder(1, 0.6, 0, 5);
      // DriveRobotEncoder(1, 0.6, 1, 6);
      // sGrabOne.setPower(0);
      // sGrabTwo.setPower(0);

      // resetHeading();
      // telemetry.update();

      // if (Purple) {
      //   turnToHeading(TURN_SPEED, -90.0);
      //   resetHeading();
      //   driveStraight(DRIVE_SPEED, 18.0, 0.0);
      //   turnToHeading(TURN_SPEED, 90.0);
      //   driveStraight(DRIVE_SPEED, 12.0, 0.0);
      // }

      // if (Green) {
      //   DriveRobot(0, 0, 5);
      // }

      // if (Red) {
      //   turnToHeading( TURN_SPEED, 90.0);
      //   resetHeading();
      //   driveStraight(DRIVE_SPEED, 18.0, 0.0);
      //   turnToHeading(TURN_SPEED, -90.0);
      //   driveStraight(DRIVE_SPEED, 12.0, 0.0);
      // }
      // Green
      // CurrentColor = Color.rgb(Color_Sensor_REV_ColorRangeSensor.red(), Color_Sensor_REV_ColorRangeSensor.green(), Color_Sensor_REV_ColorRangeSensor.blue());
      //     if (JavaUtil.colorToSaturation(CurrentColor) >= 0.6 && JavaUtil.colorToHue(CurrentColor) > 81 && JavaUtil.colorToHue(CurrentColor) < 140) {
      //     }
      // // Purple
      //   if (JavaUtil.colorToSaturation(CurrentColor) >= 0.5 && JavaUtil.colorToHue(CurrentColor) = 304) {
      //     }
      //     // Blue
      //     if (JavaUtil.colorToSaturation(CurrentColor) >= 0.6 && JavaUtil.colorToHue(CurrentColor) > 210 && JavaUtil.colorToHue(CurrentColor) < 275) {
      //     }

      //   // driveStraight(DRIVE_SPEED, 24.0, 0.0);    // Drive Forward 24"
      //   // turnToHeading( TURN_SPEED, -45.0);               // Turn  CW to -45 Degrees
      //   // holdHeading( TURN_SPEED, -45.0, 0.5);   // Hold -45 Deg heading for a 1/2 second
      //   telemetry.speak("Forward", null, null);
      //   driveStraight(DRIVE_SPEED, 15, 0);
      //   telemetry.speak("Backward", null, null);
      //   driveStraight(DRIVE_SPEED, -11, 0);
      //   telemetry.speak("Strafe Right", null, null);
      //   DriveRobotEncoder(500, 0.5, 5, 22);
      // mBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      // mBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      // mFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      // mFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      //   telemetry.speak("Forward", null, null);
      //   driveStraight(DRIVE_SPEED, 12, 0);
      //   ArmEncoder(0.5, 500 , 637);
      //   telemetry.speak("Strafe Right", null, null);
      //   DriveRobotEncoder(500, 0.5, 5, 12);

      // turnToHeading(TURN_SPEED, 90);
      // holdHeading(TURN_SPEED, 90, 0.5);
      // turnToHeading(TURN_SPEED, 0);
      // holdHeading(TURN_SPEED, 0, 0.5);

      // DriveRobot(500, 0.5, 4);
      // mBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      // mBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      // mFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      // mFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      // driveStraight(DRIVE_SPEED, 10.0, 45);
      // turnToHeading( TURN_SPEED, -45.0);
      // resetHeading();
      // holdHeading( TURN_SPEED, -45.0, 0.5);
      // ArmEncoder(0.3, 500, 500);

      // driveStraight(DRIVE_SPEED, 17.0, -45.0);  // Drive Forward 17" at -45 degrees (12"x and 12"y)
      // turnToHeading( TURN_SPEED,  45.0);               // Turn  CCW  to  45 Degrees
      // holdHeading( TURN_SPEED,  45.0, 0.5);    // Hold  45 Deg heading for a 1/2 second

      // driveStraight(DRIVE_SPEED, 17.0, 45.0);  // Drive Forward 17" at 45 degrees (-12"x and 12"y)
      // turnToHeading( TURN_SPEED,   0.0);               // Turn  CW  to 0 Degrees
      // holdHeading( TURN_SPEED,   0.0, 1.0);    // Hold  0 Deg heading for 1 second

      // driveStraight(DRIVE_SPEED,-48.0, 0.0);    // Drive in Reverse 48" (should return to approx. staring position)

      telemetry.addData("Path", "Complete");
      telemetry.update();
      sleep(1000); // Pause to display last telemetry message.
    }
  }
}
