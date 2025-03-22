package org.firstinspires.ftc.teamcode.Competition;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.Tfod;

//region imports

//endregion imports

@Autonomous(
  name = "Vision Gyro",
  group = "Competition",
  preselectTeleOp = "2022 Iterative OpMode"
)
public class Auton_Vision_Gyro extends LinearOpMode {

  //region Declaring Constants/Variables
  private CRServo Grab_One;
  private CRServo Grab_Three = null;
  private CRServo Grab_Two;
  private DcMotor Back_Left;
  private DcMotor Back_Right;
  private DcMotor Front_Left;
  private DcMotor Front_Right;
  private DcMotor Lights = null;
  private DcMotor m_Arm = null;
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
  private RevBlinkinLedDriver blinkin;
  private Tfod tfod;
  private VuforiaCurrentGame vuforiaPOWERPLAY;

  static final double COUNTS_PER_MOTOR_REV = 637.304; // eg: GoBILDA 312 RPM Yellow Jacket
  static final double DRIVE_GEAR_REDUCTION = 1.0; // No External Gearing.
  static final double DRIVE_SPEED = 0.4; // Max driving speed for better distance accuracy.
  static final double HEADING_THRESHOLD = 1.0; // How close must the heading get to the target before moving to next step.
  static final double P_DRIVE_GAIN = 0.03; // Larger is more responsive, but also less stable
  static final double P_TURN_GAIN = 0.02; // Larger is more responsive, but also less stable
  static final double TURN_SPEED = 0.2; // Max Turn speed to limit turn rate
  static final double WHEEL_DIAMETER_INCHES = 3.77953; // For figuring circumference
  static final double COUNTS_PER_INCH =
    (
      (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
      (WHEEL_DIAMETER_INCHES * 3.1415)
    );
  private BNO055IMU imu = null; // Control/Expansion Hub IMU

  Recognition recognition;
  boolean isGreenDetected;
  String Green;
  boolean isBlueDetected;
  String Blue;
  boolean isRedDetected;
  String Red;

  //endregion Declaring Constants/Variables

  //region Functions

  public void driveStraight(
    double maxDriveSpeed,
    double distance,
    double heading
  ) {
    // Ensure that the opmode is still active
    if (opModeIsActive()) {
      // Determine new target position, and pass to motor controller
      int moveCounts = (int) (distance * COUNTS_PER_INCH);
      leftTarget = Front_Left.getCurrentPosition() + moveCounts;
      leftTarget = Back_Left.getCurrentPosition() + moveCounts;
      rightTarget = Front_Right.getCurrentPosition() + moveCounts;
      rightTarget = Back_Right.getCurrentPosition() + moveCounts;

      // Set Target FIRST, then turn on RUN_TO_POSITION
      Front_Left.setTargetPosition(leftTarget);
      Back_Right.setTargetPosition(rightTarget);
      Back_Left.setTargetPosition(leftTarget);
      Front_Right.setTargetPosition(rightTarget);

      Front_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      Front_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      Back_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      Back_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

      // Set the required driving speed  (must be positive for RUN_TO_POSITION)
      // Start driving straight, and then enter the control loop
      maxDriveSpeed = Math.abs(maxDriveSpeed);
      moveRobot(maxDriveSpeed, 0);

      // keep looping while we are still active, and BOTH motors are running.

      while (
        Front_Left.isBusy() &&
        Front_Right.isBusy() &&
        Back_Left.isBusy() &&
        Back_Right.isBusy()
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
      Front_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      Front_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      Back_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      Back_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
  }

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
        Front_Left.setPower(mp);
        Back_Left.setPower(mp);
        Front_Right.setPower(mp);
        Back_Right.setPower(mp);
        break;
      case 1:
        // Drive Reverse
        Front_Left.setPower(-mp);
        Back_Left.setPower(-mp);
        Front_Right.setPower(-mp);
        Back_Right.setPower(-mp);
        break;
      case 2:
        // Turn Left
        Front_Left.setPower(-mp);
        Back_Left.setPower(-mp);
        Front_Right.setPower(mp);
        Back_Right.setPower(mp);
        break;
      case 3:
        // Turn Right
        Front_Left.setPower(mp);
        Back_Left.setPower(mp);
        Front_Right.setPower(-mp);
        Back_Right.setPower(-mp);
        break;
      case 4:
        // Strafe Left
        Front_Left.setPower(-mp);
        Back_Left.setPower(mp);
        Front_Right.setPower(mp);
        Back_Right.setPower(-mp);
        break;
      case 5:
        // Strafe Right
        Front_Left.setPower(mp);
        Back_Left.setPower(-mp);
        Front_Right.setPower(-mp);
        Back_Right.setPower(mp);
        break;
      default:
        Front_Left.setPower(0);
        Back_Left.setPower(0);
        Front_Right.setPower(0);
        Back_Right.setPower(0);
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

    Front_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Back_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Front_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Back_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    switch (nDirection) {
      case 0:
        // Drive Forward

        // Encoder Count Drive Forward - Start
        Front_Left.setPower(mp);
        Back_Left.setPower(mp);
        Front_Right.setPower(mp);
        Back_Right.setPower(mp);
        Front_Left.setTargetPosition(Ec);
        Back_Left.setTargetPosition(Ec);
        Front_Right.setTargetPosition(Ec);
        Back_Right.setTargetPosition(Ec);
        Front_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Back_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Front_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Back_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Front_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Back_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Front_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Back_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (
          !(
            !Front_Left.isBusy() &&
            !Back_Left.isBusy() &&
            !Front_Right.isBusy() &&
            !Back_Right.isBusy()
          )
        ) {}
        // Encoder Count Drive Forward - End

        break;
      case 1:
        // Drive Reverse
        Front_Left.setPower(mp);
        Back_Left.setPower(mp);
        Front_Right.setPower(mp);
        Back_Right.setPower(mp);
        Front_Left.setTargetPosition(-Ec);
        Back_Left.setTargetPosition(-Ec);
        Front_Right.setTargetPosition(-Ec);
        Back_Right.setTargetPosition(-Ec);
        Front_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Back_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Front_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Back_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Front_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Back_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Front_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Back_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (
          !(
            !Front_Left.isBusy() &&
            !Back_Left.isBusy() &&
            !Front_Right.isBusy() &&
            !Back_Right.isBusy()
          )
        ) {}
        break;
      case 2:
        // Turn Left
        Front_Left.setPower(mp);
        Back_Left.setPower(mp);
        Front_Right.setPower(mp);
        Back_Right.setPower(mp);
        Front_Left.setTargetPosition(Ec);
        Back_Left.setTargetPosition(Ec);
        Front_Right.setTargetPosition(-Ec);
        Back_Right.setTargetPosition(-Ec);
        Front_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Back_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Front_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Back_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Front_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Back_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Front_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Back_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (
          !(
            !Front_Left.isBusy() &&
            !Back_Left.isBusy() &&
            !Front_Right.isBusy() &&
            !Back_Right.isBusy()
          )
        ) {}
        break;
      case 3:
        // Turn Right
        Front_Left.setPower(mp);
        Back_Left.setPower(mp);
        Front_Right.setPower(mp);
        Back_Right.setPower(mp);
        Front_Left.setTargetPosition(-Ec);
        Back_Left.setTargetPosition(-Ec);
        Front_Right.setTargetPosition(Ec);
        Back_Right.setTargetPosition(Ec);
        Front_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Back_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Front_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Back_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Front_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Back_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Front_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Back_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (
          !(
            !Front_Left.isBusy() &&
            !Back_Left.isBusy() &&
            !Front_Right.isBusy() &&
            !Back_Right.isBusy()
          )
        ) {}
        break;
      case 4:
        // Strafe Left
        Front_Left.setPower(mp);
        Back_Left.setPower(mp);
        Front_Right.setPower(mp);
        Back_Right.setPower(mp);
        Front_Left.setTargetPosition(-Ec);
        Back_Left.setTargetPosition(Ec);
        Front_Right.setTargetPosition(Ec);
        Back_Right.setTargetPosition(-Ec);
        Front_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Back_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Front_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Back_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Front_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Back_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Front_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Back_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (
          !(
            !Front_Left.isBusy() &&
            !Back_Left.isBusy() &&
            !Front_Right.isBusy() &&
            !Back_Right.isBusy()
          )
        ) {}
        break;
      case 5:
        // Strafe Right
        Front_Left.setPower(mp);
        Back_Left.setPower(mp);
        Front_Right.setPower(mp);
        Back_Right.setPower(mp);
        Front_Left.setTargetPosition(Ec);
        Back_Left.setTargetPosition(-Ec);
        Front_Right.setTargetPosition(-Ec);
        Back_Right.setTargetPosition(Ec);
        Front_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Back_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Front_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Back_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // Front_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Back_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Front_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Back_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (
          !(
            !Front_Left.isBusy() &&
            !Back_Left.isBusy() &&
            !Front_Right.isBusy() &&
            !Back_Right.isBusy()
          )
        ) {}

        break;
      default:
        Front_Left.setPower(0);
        Back_Left.setPower(0);
        Front_Right.setPower(0);
        Back_Right.setPower(0);
        break;
    }

    sleep(ms);
    telemetry.addData("Ec", Ec);
    telemetry.update();
  }

  public void ArmEncoder(double mp, int eC, int ms) {
    m_Arm.setPower(mp);
    m_Arm.setTargetPosition(eC);
    m_Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    while (!(!m_Arm.isBusy())) {}
    sleep(ms);
  }

  private void displayInfo(int i) {
    // Display the location of the top left corner
    // of the detection boundary for the recognition
    telemetry.addData(
      "Label: " +
      recognition.getLabel() +
      ", Confidence: " +
      recognition.getConfidence(),
      "X: " +
      Math.round(
        JavaUtil.averageOfList(
          JavaUtil.createListWith(
            Double.parseDouble(JavaUtil.formatNumber(recognition.getLeft(), 0)),
            Double.parseDouble(JavaUtil.formatNumber(recognition.getRight(), 0))
          )
        )
      ) +
      ", Y: " +
      Math.round(
        JavaUtil.averageOfList(
          JavaUtil.createListWith(
            Double.parseDouble(JavaUtil.formatNumber(recognition.getTop(), 0)),
            Double.parseDouble(
              JavaUtil.formatNumber(recognition.getBottom(), 0)
            )
          )
        )
      )
    );
    if (recognition.getLabel().equals(Green)) {
      isGreenDetected = true;
      telemetry.addData("Object Detected", "Green");
    } else {
      isGreenDetected = false;
    }
    if (recognition.getLabel().equals(Blue)) {
      isBlueDetected = true;
      telemetry.addData("Object Detected", "Blue");
    } else {
      isBlueDetected = false;
    }
    if (recognition.getLabel().equals(Red)) {
      isRedDetected = true;
      telemetry.addData("Object Detected", "Red");
    } else {
      isRedDetected = false;
    }
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

    Front_Left.setPower(leftSpeed);
    Back_Left.setPower(leftSpeed);
    Back_Right.setPower(rightSpeed);
    Front_Right.setPower(rightSpeed);
  }

  private void sendTelemetry(boolean straight) {
    if (straight) {
      telemetry.addData("Motion", "Drive Straight");
      telemetry.addData("Target Pos L:R", "%7d:%7d", leftTarget, rightTarget);
      telemetry.addData(
        "Actual Pos L:R",
        "%7d:%7d",
        Front_Left.getCurrentPosition(),
        Front_Right.getCurrentPosition(),
        Back_Right.getCurrentPosition()
      );
      Back_Left.getCurrentPosition();
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

  //endregion Functions

  @Override
  public void runOpMode() {
    List<Recognition> recognitions;
    int index;
    vuforiaPOWERPLAY = new VuforiaCurrentGame();

    // blinkin = hardwareMap.get(RevBlinkinLedDriver, "blinkin");
    Back_Right = hardwareMap.get(DcMotor.class, "Back_Right");
    Back_Left = hardwareMap.get(DcMotor.class, "Back_Left");
    Front_Right = hardwareMap.get(DcMotor.class, "Front_Right");
    Front_Left = hardwareMap.get(DcMotor.class, "Front_Left");
    Lights = hardwareMap.get(DcMotor.class, "Lights");
    m_Arm = hardwareMap.get(DcMotor.class, "m_Arm");
    Grab_One = hardwareMap.get(CRServo.class, "Grab_One");
    Grab_Two = hardwareMap.get(CRServo.class, "Grab_Two");
    Grab_Three = hardwareMap.get(CRServo.class, "Grab_Three");
    tfod = new Tfod();

    // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
    // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
    // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
    Front_Left.setDirection(DcMotor.Direction.REVERSE);
    Back_Left.setDirection(DcMotor.Direction.REVERSE);
    m_Arm.setDirection(DcMotor.Direction.REVERSE);
    // Sample TFOD Op Mode using a Custom Model
    // Initialize Vuforia to provide TFOD with camera
    // images.
    // The following block uses the device's back camera.
    // The following block uses a webcam.
    vuforiaPOWERPLAY.initialize(
      "", // vuforiaLicenseKey
      hardwareMap.get(WebcamName.class, "Webcam 1"), // cameraName
      "", // webcamCalibrationFilename
      false, // useExtendedTracking
      false, // enableCameraMonitoring
      VuforiaLocalizer.Parameters.CameraMonitorFeedback.NONE, // cameraMonitorFeedback
      0, // dx
      0, // dy
      0, // dz
      AxesOrder.XZY, // axesOrder
      90, // firstAngle
      90, // secondAngle
      0, // thirdAngle
      true
    ); // useCompetitionFieldTargetLocations
    // Initialize TFOD before waitForStart.
    // Use the Manage page to upload your custom model.
    // In the next block, replace
    // YourCustomModel.tflite with the name of your
    // custom model.

    // define initialization values for IMU, and then initialize it.
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    imu = hardwareMap.get(BNO055IMU.class, "imu");
    imu.initialize(parameters);

    // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
    Back_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Back_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Front_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Front_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Front_Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    Front_Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    Back_Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    Back_Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    Red = "Red";
    Blue = "Blue";
    Green = "Green";
    isRedDetected = false;
    isBlueDetected = false;
    isGreenDetected = false;
    // Set isModelTensorFlow2 to true if you used a TensorFlow
    // 2 tool, such as ftc-ml, to create the model.
    //
    // Set isModelQuantized to true if the model is
    // quantized. Models created with ftc-ml are quantized.
    //
    // Set inputSize to the image size corresponding to the model.
    // If your model is based on SSD MobileNet v2
    // 320x320, the image size is 300 (srsly!).
    // If your model is based on SSD MobileNet V2 FPNLite 320x320, the image size is 320.
    // If your model is based on SSD MobileNet V1 FPN 640x640 or
    // SSD MobileNet V2 FPNLite 640x640, the image size is 640.
    tfod.useModelFromFile(
      "model_20221121_182459.tflite",
      JavaUtil.createListWith(Blue, Green, Red),
      true,
      true,
      300
    );
    tfod.initialize(vuforiaPOWERPLAY, (float) 0.7, true, true);
    tfod.setClippingMargins(0, 80, 0, 0);
    tfod.activate();
    // Enable following block to zoom in on target.
    tfod.setZoom(1.5, 4 / 3);
    telemetry.addData("DS preview on/off", "3 dots, Camera Stream");

    // Wait for the game to start (Display Gyro value while waiting)
    while (opModeInInit()) {
      telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
      telemetry.update();
    }
    telemetry.addData("Action needed", "Please press the start triangle");
    telemetry.update();
    // Wait for start command from Driver Station.
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        Lights.setPower(1);
        // Get a list of recognitions from TFOD.
        recognitions = tfod.getRecognitions();
        // If list is empty, inform the user. Otherwise, go
        // through list and display info for each recognition.
        if (JavaUtil.listLength(recognitions) == 0) {
          telemetry.addData("TFOD", "No items detected.");
        } else {
          index = 0;
          // Iterate through list and call a function to
          // display info for each recognized object.
          for (Recognition recognition_item : recognitions) {
            recognition = recognition_item;
            // Display info.
            displayInfo(index);
            // Increment index.
            index = index + 1;
          }
        }
        telemetry.update();
        if (isGreenDetected) {
          Grab_One.setPower(1);
        }
        if (isBlueDetected) {
          Grab_Two.setPower(1);
        }
        if (isRedDetected) {
          Lights.setPower(0);
        }
      }
    }
    // Deactivate TFOD.
    tfod.deactivate();

    vuforiaPOWERPLAY.close();
    tfod.close();
    // Set the encoders for closed loop speed control, and reset the heading.
    Back_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    Back_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    Front_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    Front_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    resetHeading();

    ArmEncoder(0.5, 200, 500);
    driveStraight(DRIVE_SPEED, 5.0, 0.0);
    driveStraight(DRIVE_SPEED, -2.0, 0.0);
    turnToHeading(TURN_SPEED, -90.0);
    resetHeading();
    telemetry.update();
    driveStraight(DRIVE_SPEED, 18.0, 0.0);
    resetHeading();
    telemetry.update();
    turnToHeading(TURN_SPEED, 90.0);
    resetHeading();
    telemetry.update();
    driveStraight(DRIVE_SPEED, 40.0, 0.0);
    resetHeading();
    telemetry.update();
    DriveRobotEncoder(500, 0.5, 4, 12.55);
    ArmEncoder(0.5, 3000, 500);
    Grab_One.setPower(-1);
    Grab_Two.setPower(-1);
    DriveRobotEncoder(500, 0.3, 0, 6);
    DriveRobotEncoder(300, 0.3, 1, 6);

    resetHeading();
    telemetry.update();

    telemetry.addData("Path", "Complete");
    telemetry.update();
    sleep(1000); // Pause to display last telemetry message.
  }
}
