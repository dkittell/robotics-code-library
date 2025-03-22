package org.firstinspires.ftc.teamcode.Competition.Auton;

//region imports
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
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Tfod;

//endregion imports

@Autonomous(
  name = "Auton B1R2",
  group = "Competition",
  preselectTeleOp = "2022 Iterative OpMode"
)
public class Auton_B1R2 extends LinearOpMode {

  //region Declaring Constants/Variables
  private CRServo sGrabOne = null;
  private CRServo sGrabThree = null;
  private CRServo sGrabTwo = null;
  private DcMotor mBackLeft = null;
  private DcMotor mBackRight = null;
  private DcMotor mFrontLeft = null;
  private DcMotor mFrontRight = null;
  private DcMotor mLights = null;
  private DcMotor mArm = null;
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
  private RevBlinkinLedDriver blinkin = null;
  private Tfod Tfod = null;
  private VuforiaCurrentGame vuforiaPOWERPLAY = null;

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

  private static final String VUFORIA_KEY =
    "AURxXKH/////AAABmXA804Jo1kAesHgmilL17rdCzb/63G1nuVbHyDwVTC4CxnaHgV3t8f0/nr552b09dNHDJc4O8Lt4F0DFJ+WxwHVIKpAak7X/0+OwWXruINDzQ27ad/esrI6zDPFKmbdvSpq9HqIlrzMxbHxrPBEkKeXufyPjdkAHzR1IYAK5ikNoHtPc/OYQObtjWp9Mzw+oMwQJ8O0SfXQF+Z7R16TGWqXCJUQmnscXWDGWjeVaWnuXXj3qhXC1sOLF/xTC0B35miAG09T2lljvkmNlig1pzMnqqlboKcQlCg701NzO69E/rZvDH83uzMZCKVuBzZ0+PYvuQByQFtUezXCyu2qJ1sTQ/YrqrKe086bUDfEcBFTW";
  private VuforiaLocalizer vuforia;
  private TFObjectDetector tfod;

  private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
  // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";

  private static final String[] LABELS = { "1 Bolt", "2 Bulb", "3 Panel" };

  Recognition recognition = null;
  boolean BoltDetected = false;
  boolean PanelDetected = false;
  boolean BulbDetected = false;

  //endregion Declaring Constants/Variables

  @Override
  public void runOpMode() {
    List<Recognition> recognitions;
    int index;
    String Bolt;
    String Panel;
    String Bulb;
    vuforiaPOWERPLAY = new VuforiaCurrentGame();

    // blinkin = hardwareMap.get(RevBlinkinLedDriver, "blinkin");
    mBackRight = hardwareMap.get(DcMotor.class, "mBackRight");
    mBackLeft = hardwareMap.get(DcMotor.class, "mBackLeft");
    mFrontRight = hardwareMap.get(DcMotor.class, "mFrontRight");
    mFrontLeft = hardwareMap.get(DcMotor.class, "mFrontLeft");
    mLights = hardwareMap.get(DcMotor.class, "mLights");
    mArm = hardwareMap.get(DcMotor.class, "mArm");
    sGrabOne = hardwareMap.get(CRServo.class, "sGrabOne");
    sGrabTwo = hardwareMap.get(CRServo.class, "sGrabTwo");
    sGrabThree = hardwareMap.get(CRServo.class, "sGrabThree");
    //tfod = new Tfod();

    // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
    // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
    // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
    mFrontLeft.setDirection(DcMotor.Direction.REVERSE);
    mBackLeft.setDirection(DcMotor.Direction.REVERSE);
    mArm.setDirection(DcMotor.Direction.REVERSE);
    // Sample TFOD Op Mode using a Custom Model
    // Initialize Vuforia to provide TFOD with camera
    // images.
    // The following block uses the device's back camera.
    // The following block uses a webcam.
    // vuforiaPOWERPLAY.initialize(
    //   "AURxXKH/////AAABmXA804Jo1kAesHgmilL17rdCzb/63G1nuVbHyDwVTC4CxnaHgV3t8f0/nr552b09dNHDJc4O8Lt4F0DFJ+WxwHVIKpAak7X/0+OwWXruINDzQ27ad/esrI6zDPFKmbdvSpq9HqIlrzMxbHxrPBEkKeXufyPjdkAHzR1IYAK5ikNoHtPc/OYQObtjWp9Mzw+oMwQJ8O0SfXQF+Z7R16TGWqXCJUQmnscXWDGWjeVaWnuXXj3qhXC1sOLF/xTC0B35miAG09T2lljvkmNlig1pzMnqqlboKcQlCg701NzO69E/rZvDH83uzMZCKVuBzZ0+PYvuQByQFtUezXCyu2qJ1sTQ/YrqrKe086bUDfEcBFTW", // vuforiaLicenseKey
    //   hardwareMap.get(WebcamName.class, "wcVision"), // cameraName
    //   "", // webcamCalibrationFilename
    //   false, // useExtendedTracking
    //   false, // enableCameraMonitoring
    //   VuforiaLocalizer.Parameters.CameraMonitorFeedback.NONE, // cameraMonitorFeedback
    //   0, // dx
    //   0, // dy
    //   0, // dz
    //   AxesOrder.XZY, // axesOrder
    //   90, // firstAngle
    //   90, // secondAngle
    //   0, // thirdAngle
    //   true
    // ); // useCompetitionFieldTargetLocations
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
    mBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    mBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    mFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    mFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    mFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    mFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    mBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    mBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    // Horse = "Horse";
    // Lego = "Lego";
    // Tobar = "Tobar";
    BoltDetected = false;
    PanelDetected = false;
    BulbDetected = false;
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

    // Enable following block to zoom in on target.
    telemetry.addData("DS preview on/off", "3 dots, Camera Stream");

    // // Wait for the game to start (Display Gyro value while waiting)
    while (opModeInInit()) {
      telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
      telemetry.update();
      mLights.setPower(0);
      // tfod.setZoom(1, 16 / 9);
      // Tfod.activate();
      // initVuforia();
      // initTfod();
      // if (tfod != null) {
      //   List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
      //   if (updatedRecognitions != null) {
      //     telemetry.addData("# Objects Detected", updatedRecognitions.size());

      //     // step through the list of recognitions and display image position/size information for each one
      //     // Note: "Image number" refers to the randomized image orientation/number
      //     for (Recognition recognition : updatedRecognitions) {
      //       double col = (recognition.getLeft() + recognition.getRight()) / 2;
      //       double row = (recognition.getTop() + recognition.getBottom()) / 2;
      //       double width = Math.abs(
      //         recognition.getRight() - recognition.getLeft()
      //       );
      //       double height = Math.abs(
      //         recognition.getTop() - recognition.getBottom()
      //       );

      //       telemetry.addData("", " ");
      //       telemetry.addData(
      //         "Image",
      //         "%s (%.0f %% Conf.)",
      //         recognition.getLabel(),
      //         recognition.getConfidence() * 100
      //       );
      //       telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
      //       telemetry.addData(
      //         "- Size (Width/Height)",
      //         "%.0f / %.0f",
      //         width,
      //         height
      //       );
      //     }
      //     telemetry.update();
      //   }
      // }
    }
    telemetry.addData("Action needed", "Please press the start triangle");
    telemetry.update();
    // // Wait for start command from Driver Station.

    waitForStart();
    if (opModeIsActive()) {
      // Set the encoders for closed loop speed control, and reset the heading.
      mBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      mBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      mFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      mFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      // if (tfod != null) {
      //         List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
      //         if (updatedRecognitions != null) {
      //           telemetry.addData("# Objects Detected", updatedRecognitions.size());

      //           // step through the list of recognitions and display image position/size information for each one
      //           // Note: "Image number" refers to the randomized image orientation/number
      //           for (Recognition recognition : updatedRecognitions) {
      //             double col = (recognition.getLeft() + recognition.getRight()) / 2;
      //             double row = (recognition.getTop() + recognition.getBottom()) / 2;
      //             double width = Math.abs(
      //               recognition.getRight() - recognition.getLeft()
      //             );
      //             double height = Math.abs(
      //               recognition.getTop() - recognition.getBottom()
      //             );

      //             telemetry.addData("", " ");
      //             telemetry.addData(
      //               "Image",
      //               "%s (%.0f %% Conf.)",
      //               recognition.getLabel(),
      //               recognition.getConfidence() * 100
      //             );
      //             telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
      //             telemetry.addData(
      //               "- Size (Width/Height)",
      //               "%.0f / %.0f",
      //               width,
      //               height
      //             );
      //           }
      //           telemetry.update();
      //         }
      //       }

      sGrabOne.setPower(1); // grabs cone fully
      sGrabTwo.setPower(-1); // grabs cone fully

      ArmEncoder(1, 200, 1); // Raise cone off the ground to drive with preload

      sleep(250); // used to give robot time to do something

      DriveRobotEncoder(1, 0.5, 4, 1); // Strafe away from junction
      resetHeading();
      ArmEncoder(1, 1000, 1); // Move arm above signal sleeve cone

      sGrabOne.setPower(0); // Stop servos
      sGrabTwo.setPower(0); // Stop servos

      // // Set robot to coast/float for driving to the high junction then after it set it to brake
      // // mFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
      // // mFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
      // // mBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
      // // mBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
      driveStraight(DRIVE_SPEED, 50, 0); // Drive straight to get to the high junction44
      mFrontLeft.setPower(0); // Allow Robot Time
      mFrontRight.setPower(0); // Allow Robot Time
      mBackLeft.setPower(0); // Allow Robot Time
      mBackRight.setPower(0); // Allow Robot Time
      sleep(250); // Allow Robot Time
      // mFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      // mFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      // mBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      // mBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

      // resetHeading(); // dec 5 2022
      driveStraight(0.5, 2, 0); // slight boost to allow smooth stop
      // resetHeading(); //dec 5 2022
      sleep(1000);
      driveStraight(0.7, -11.5, 0); // backward for lineing
      // resetHeading(); //dec 5 2022
      turnToHeading(TURN_SPEED, -37); // turn to line up with High Junction
      resetHeading();
      ArmEncoder(1, 3000, 1); // move arm up to high junction
      resetHeading();
      telemetry.update();

      driveStraight(0.6, 8.5, 0); // forward to move over high junction
      resetHeading();
      sleep(1000); // give robot time
      sGrabOne.setPower(-1); // drop cone on high junction
      sGrabTwo.setPower(1); // drop cone on high junction
      sleep(250);
      resetHeading();
      driveStraight(0.6, -7, 0); // move backward away from junction
      ArmEncoder(1, 1000, 1); // move arm up to prepare for stack
      resetHeading();

      // //end of preloaded cone, now moving to stack
      turnToHeading(TURN_SPEED, 125); // turn to line up for stack
      resetHeading();
      DriveRobotEncoder(1, 0.5, 4, 1.25); // strafe to line up with stack
      resetHeading();
      driveStraight(0.7, 23, 0); // forward over the stack
      resetHeading();
      sGrabOne.setPower(1); // grab cone off of stack
      sGrabTwo.setPower(-1); // grab cone off of stack
      sleep(1500);
      resetHeading();
      ArmEncoder(0.6, 500, 1); // move arm down to grab cone
      sGrabOne.setPower(0); // stop servos
      sGrabTwo.setPower(0); // stop servos
      resetHeading();
      ArmEncoder(0.6, 1000, 1); // move arm down to pick up from stack
      resetHeading();
      sGrabOne.setPower(0); // stop servos again
      sGrabTwo.setPower(0); // stop servos again
      driveStraight(DRIVE_SPEED, -20, 0); // drive backwards away from stack
      resetHeading();
      turnToHeading(TURN_SPEED, 137); // turn towards medium junction
      resetHeading();
      driveStraight(DRIVE_SPEED, 4.5, 0); // move forward to line up with junction
      resetHeading();
      ArmEncoder(0.7, 2200, 500); // move arm up to line up with medium junction
      resetHeading();
      driveStraight(0.5, 4.5, 0); // move forward over the junction
      sGrabOne.setPower(-1); // reverse servos to place
      sGrabTwo.setPower(1); // reverese servos to place
      sleep(1000);
      resetHeading();
      driveStraight(0.5, -6.5, 0); // move backward from the junction
      resetHeading();
      turnToHeading(TURN_SPEED, 50); // turn to park in zone 2
      ArmEncoder(0.7, 40, 500); // move arm up to line up with medium junction
      // mArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      // resetHeading();
      // turnToHeading(TURN_SPEED, 90.0);
      // ArmEncoder(0.6, 1000, 1);
      // resetHeading();
      // driveStraight(DRIVE_SPEED, 20, 0.0);
      // DriveRobotEncoder(1, 0.4, 5, 2);
      // resetHeading();
      // sGrabOne.setPower(1);
      // sGrabTwo.setPower(-1);
      // sleep(1000);
      // driveStraight(0.5, 1, 0.0);
      // ArmEncoder(0.6, 500, 1); // move arm down to grab cone
      // sGrabOne.setPower(0);
      // sGrabTwo.setPower(0);
      // resetHeading();
      // telemetry.update();
      // ArmEncoder(0.6, 2000, 1);
      // driveStraight(DRIVE_SPEED, -24, 0.0); // Drive backwards away from stack
      // resetHeading();
      // telemetry.update();
      // resetHeading();
      // turnToHeading(TURN_SPEED, 90);
      // DriveRobotEncoder(1, 0.4, 4, 7);
      // ArmEncoder(0.6, 2229, 1);
      // resetHeading();
      // driveStraight(DRIVE_SPEED, 6, 0);
      // resetHeading();
      // sGrabOne.setPower(-1);
      // sGrabTwo.setPower(1);
      // sleep(500);
      // resetHeading();
      // telemetry.update();
      // sGrabOne.setPower(0);
      // sGrabTwo.setPower(0);
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

  public void ArmEncoder(double mp, int eC, int ms) {
    mArm.setPower(mp);
    mArm.setTargetPosition(eC);
    mArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    while (!(!mArm.isBusy())) {}
    sleep(ms);
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

  public void manipulateCone(int sP, int ms) {
    sGrabOne.setPower(sP);
    sGrabTwo.setPower(sP);
    sleep(ms);
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
  // private void initTfod() {
  //     int tfodMonitorViewId = hardwareMap.appContext
  //       .getResources()
  //       .getIdentifier(
  //         "tfodMonitorViewId",
  //         "id",
  //         hardwareMap.appContext.getPackageName()
  //       );
  //     TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(
  //       tfodMonitorViewId
  //     );
  //     tfodParameters.minResultConfidence = 0.75f;
  //     tfodParameters.isModelTensorFlow2 = true;
  //     tfodParameters.inputSize = 300;
  //     tfod =
  //       ClassFactory
  //         .getInstance()
  //         .createTFObjectDetector(tfodParameters, vuforia);
  //     // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
  //     // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
  //     tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
  //   }
  //   public void displayInfo(int i) {
  //     // Display the location of the top left corner
  //     // of the detection boundary for the recognition
  //     telemetry.addData(
  //       "Label: " +
  //       recognition.getLabel() +
  //       ", Confidence: " +
  //       recognition.getConfidence(),
  //       "X: " +
  //       Math.round(
  //         JavaUtil.averageOfList(
  //           JavaUtil.createListWith(
  //             Double.parseDouble(JavaUtil.formatNumber(recognition.getLeft(), 0)),
  //             Double.parseDouble(JavaUtil.formatNumber(recognition.getRight(), 0))
  //           )
  //         )
  //       ) +
  //       ", Y: " +
  //       Math.round(
  //         JavaUtil.averageOfList(
  //           JavaUtil.createListWith(
  //             Double.parseDouble(JavaUtil.formatNumber(recognition.getTop(), 0)),
  //             Double.parseDouble(
  //               JavaUtil.formatNumber(recognition.getBottom(), 0)
  //             )
  //           )
  //         )
  //       )
  //     );
  //     if (recognition.getLabel().equals("Bulb")) {
  //       BulbDetected = true;
  //       telemetry.addData("Object Detected", "BULB");
  //     } else {
  //       BulbDetected = false;
  //   }

  //   if (recognition.getLabel().equals("Bolt")) {
  //     BoltDetected = true;
  //     telemetry.addData("Object Detected", "BOLT");
  //   } else {
  //     BoltDetected= false;
  //     if (recognition.getLabel().equals("Panel")) {
  //     PanelDetected = true;
  //     telemetry.addData("Object Detected", "PANEL");
  //   } else {
  //     PanelDetected= false;
  //   // private void initVuforia() {
  //   /*
  //   * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
  //   */
  //   VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

  //   parameters.vuforiaLicenseKey = VUFORIA_KEY;
  //   parameters.cameraName = hardwareMap.get(WebcamName.class, "wcVision");

  //   //  Instantiate the Vuforia engine
  //   vuforia = ClassFactory.getInstance().createVuforia(parameters);
  // }

}
//endregion Functions
