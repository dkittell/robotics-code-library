package org.firstinspires.ftc.teamcode.Competition;

//region imports
import android.graphics.Color;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.text.SimpleDateFormat;
import java.util.Date;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//endregion imports

@Autonomous(
  name = "Blue 2 / Red 1",
  group = "Competition",
  preselectTeleOp = "2022 Iterative OpMode"
)
public class Blue2 extends LinearOpMode {

  //region Declaring Constants/Variables
  private RevBlinkinLedDriver blinkin;
  /* Declare OpMode members. */
  private DcMotor Back_Left = null;
  private DcMotor Back_Right = null;
  private DcMotor Front_Left = null;
  private DcMotor Front_Right = null;
  private BNO055IMU imu = null; // Control/Expansion Hub IMU
  private DcMotor m_Arm = null;
  private ColorSensor Color_Sensor;
  private CRServo Grab_One = null;
  private CRServo Grab_Two = null;
  private CRServo Grab_Three = null;
  private ColorSensor Color_Sensor_REV_ColorRangeSensor;

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
  public int CurrentColor;

  public boolean Green;
  public boolean Pink;
  public boolean Purple;

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
  static final double DRIVE_SPEED = 0.4; // Max driving speed for better distance accuracy.
  static final double TURN_SPEED = 0.2; // Max Turn speed to limit turn rate
  static final double HEADING_THRESHOLD = 1.0; // How close must the heading get to the target before moving to next step.
  // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
  // Define the Proportional control coefficient (or GAIN) for "heading control".
  // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
  // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
  // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
  static final double P_TURN_GAIN = 0.02; // Larger is more responsive, but also less stable
  static final double P_DRIVE_GAIN = 0.03; // Larger is more responsive, but also less stable

  //endregion Declaring Constants/Variables

  @Override
  public void runOpMode() {
    // region runOpMode
    // region Initialization

    int gain = 2;
    NormalizedRGBA normalizedColors;
    int color;
    float hue;
    float saturation;
    float value;

    // Initialize the drive system variables.
    blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

    Back_Right = hardwareMap.get(DcMotor.class, "Back_Right");
    Back_Left = hardwareMap.get(DcMotor.class, "Back_Left");
    Front_Right = hardwareMap.get(DcMotor.class, "Front_Right");
    Front_Left = hardwareMap.get(DcMotor.class, "Front_Left");
    m_Arm = hardwareMap.get(DcMotor.class, "m_Arm");

    ColorSensor ColorSensor_REV_ColorRangeSensor = hardwareMap.get(
      ColorSensor.class,
      "Color_Sensor"
    );
    Grab_One = hardwareMap.get(CRServo.class, "Grab_One");
    Grab_Two = hardwareMap.get(CRServo.class, "Grab_Two");
    Grab_Three = hardwareMap.get(CRServo.class, "Grab_Three");
    // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
    // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
    // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
    Front_Left.setDirection(DcMotor.Direction.REVERSE);
    Back_Left.setDirection(DcMotor.Direction.REVERSE);
    m_Arm.setDirection(DcMotor.Direction.REVERSE);

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
    Back_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Back_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Front_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Front_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Front_Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    Front_Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    Back_Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    Back_Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    // Wait for the game to start (Display Gyro value while waiting)
    while (opModeInInit()) {
      telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
      telemetry.update();
    }
    telemetry.addData("Action needed", "Please press the start triangle");
    telemetry.update();
    // endregion Initialization
    waitForStart();
    if (opModeIsActive()) {
      // Set the encoders for closed loop speed control, and reset the heading.
      Back_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      Back_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      Front_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      Front_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      resetHeading();
      // region Get Color 1 Testing
      // CurrentColor =
      //   Color.rgb(
      //     Color_Sensor_REV_ColorRangeSensor.red(),
      //     Color_Sensor_REV_ColorRangeSensor.green(),
      //     Color_Sensor_REV_ColorRangeSensor.blue()
      //   );
      // Green =
      //   (
      //     JavaUtil.colorToSaturation((int) CurrentColor) >= 0.6 &&
      //     (
      //       JavaUtil.colorToHue((int) CurrentColor) > 81 ||
      //       JavaUtil.colorToHue((int) CurrentColor) < 140
      //     )
      //   );
      // telemetry.addData("Green", Green);
      // telemetry.update();
      // Pink =
      //   (
      //     JavaUtil.colorToSaturation((int) CurrentColor) <= 0.4 &&
      //     (
      //       JavaUtil.colorToHue((int) CurrentColor) >= 351 ||
      //       JavaUtil.colorToHue((int) CurrentColor) <= 360
      //     )
      //   );
      // telemetry.addData("Pink", Pink);
      // telemetry.update();
      // Purple =
      //   (
      //     JavaUtil.colorToSaturation((int) CurrentColor) <= 0.4 &&
      //     (
      //       JavaUtil.colorToHue((int) CurrentColor) >= 317 ||
      //       JavaUtil.colorToHue((int) CurrentColor) <= 270
      //     )
      //   );
      // telemetry.addData("Purple", Purple);
      // telemetry.update();
      // endregion Get Color 1 Testing

      // region Get Color 2 Testing
      // Display distance info.
      telemetry.addData(
        "Dist to tgt (cm)",
        ((DistanceSensor) ColorSensor_REV_ColorRangeSensor).getDistance(
            DistanceUnit.CM
          )
      );
      // Display reflected light.
      telemetry.addData(
        "Light detected",
        (
          (OpticalDistanceSensor) ColorSensor_REV_ColorRangeSensor
        ).getLightDetected()
      );

      ((NormalizedColorSensor) ColorSensor_REV_ColorRangeSensor).setGain(gain);
      telemetry.addData(
        "Gain",
        ((NormalizedColorSensor) ColorSensor_REV_ColorRangeSensor).getGain()
      );

      // Read color from the sensor.
      normalizedColors =
        (
          (NormalizedColorSensor) ColorSensor_REV_ColorRangeSensor
        ).getNormalizedColors();
      telemetry.addData(
        "Red",
        Double.parseDouble(JavaUtil.formatNumber(normalizedColors.red, 3))
      );
      telemetry.addData(
        "Green",
        Double.parseDouble(JavaUtil.formatNumber(normalizedColors.green, 3))
      );
      telemetry.addData(
        "Blue",
        Double.parseDouble(JavaUtil.formatNumber(normalizedColors.blue, 3))
      );
      // Convert RGB values to Hue, Saturation, and Value.
      // See https://en.wikipedia.org/wiki/HSL_and_HSV for details on HSV color model.
      color = normalizedColors.toColor();
      hue = JavaUtil.colorToHue(color);
      saturation = JavaUtil.colorToSaturation(color);
      value = JavaUtil.colorToValue(color);
      telemetry.addData(
        "Hue",
        Double.parseDouble(JavaUtil.formatNumber(hue, 0))
      );
      telemetry.addData(
        "Saturation",
        Double.parseDouble(JavaUtil.formatNumber(saturation, 3))
      );
      telemetry.addData(
        "Value",
        Double.parseDouble(JavaUtil.formatNumber(value, 3))
      );
      telemetry.addData(
        "Alpha",
        Double.parseDouble(JavaUtil.formatNumber(normalizedColors.alpha, 3))
      );
      // Show the color on the Robot Controller screen.
      JavaUtil.showColor(hardwareMap.appContext, color);
      // Use hue to determine if it's red, green, blue, etc..
      if (hue < 30) {
        telemetry.addData("Color", "Red");
      } else if (hue < 60) {
        telemetry.addData("Color", "Orange");
      } else if (hue < 90) {
        telemetry.addData("Color", "Yellow");
      } else if (hue < 150) {
        telemetry.addData("Color", "Green");
      } else if (hue < 225) {
        telemetry.addData("Color", "Blue");
      } else if (hue < 350) {
        telemetry.addData("Color", "purple");
      } else {
        telemetry.addData("Color", "Red");
      }
      // Check to see if it might be black or white.
      if (saturation < 0.2) {
        telemetry.addData("Check Sat", "Is surface white?");
      }
      telemetry.update();
      if (value < 0.16) {
        telemetry.addData("Check Val", "Is surface black?");
      }

      // endregion Get Color 2 Testing

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
      // ArmEncoder(0.5, 200, 500);
      // driveStraight(DRIVE_SPEED, 5.0, 0.0);
      // driveStraight(DRIVE_SPEED, -2.0, 0.0);
      // turnToHeading(TURN_SPEED, 90.0);
      // resetHeading();
      // telemetry.update();
      // driveStraight(DRIVE_SPEED, 18.0, 0.0);
      // resetHeading();
      // telemetry.update();
      // turnToHeading(TURN_SPEED, -90.0);
      // resetHeading();
      // telemetry.update();
      // driveStraight(DRIVE_SPEED, 40.0, 0.0);
      // resetHeading();
      // telemetry.update();
      // DriveRobotEncoder(500, 0.5, 5, 12.55);
      // ArmEncoder(0.5, 3000, 500);
      // Grab_One.setPower(-1);
      // Grab_Two.setPower(-1);
      // DriveRobotEncoder(500, 0.3, 0, 6);
      // DriveRobotEncoder(300, 0.3, 1, 6);

      resetHeading();
      telemetry.update();

      if (Purple) {
        Back_Left.setPower(0.5);
        Back_Right.setPower(-0.5);
        Front_Left.setPower(-0.5);
        Front_Right.setPower(0.5);
        sleep(500);
      }

      if (Green) {
        Back_Left.setPower(0.5);
        Back_Right.setPower(-0.5);
        Front_Left.setPower(-0.5);
        Front_Right.setPower(0.5);
        sleep(500);
      }

      if (Pink) {
        Back_Left.setPower(-0.5);
        Back_Right.setPower(0.5);
        Front_Left.setPower(0.5);
        Front_Right.setPower(-0.5);
        sleep(500);
      }

      //   // driveStraight(DRIVE_SPEED, 24.0, 0.0);    // Drive Forward 24"
      //   // turnToHeading( TURN_SPEED, -45.0);               // Turn  CW to -45 Degrees
      //   // holdHeading( TURN_SPEED, -45.0, 0.5);   // Hold -45 Deg heading for a 1/2 second
      //   telemetry.speak("Forward", null, null);
      //   driveStraight(DRIVE_SPEED, 15, 0);
      //   telemetry.speak("Backward", null, null);
      //   driveStraight(DRIVE_SPEED, -11, 0);
      //   telemetry.speak("Strafe Right", null, null);
      //   DriveRobotEncoder(500, 0.5, 5, 22);
      // Back_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      // Back_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      // Front_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      // Front_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
      // Back_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      // Back_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      // Front_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      // Front_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
    // endregion runOpMode
  }

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
  public void LeftStrafeToPosition(double eC, double mp, int nDirection) {
    switch (nDirection) {
      case 0:
        // int moveCounts = (int)(distance * COUNTS_PER_INCH);
        resetHeading();

        Front_Left.setPower(mp);
        Front_Right.setPower(mp);
        Back_Left.setPower(mp);
        Back_Right.setPower(mp);

        Front_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Front_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Back_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Back_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // leftTarget = Front_Left.getCurrentPosition() + moveCounts;
        // leftTarget = Back_Left.getCurrentPosition() + moveCounts;
        // rightTarget = Front_Right.getCurrentPosition() + moveCounts;
        // rightTarget = Back_Right.getCurrentPosition() + moveCounts;

        Front_Left.setTargetPosition((int) -eC);
        Back_Right.setTargetPosition((int) -eC);
        Back_Left.setTargetPosition((int) eC);
        Front_Right.setTargetPosition((int) eC);

        Front_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Front_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Back_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Back_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        moveRobot(0, 0);
        while (
          !(
            !Front_Right.isBusy() &&
            !Front_Left.isBusy() &&
            !Back_Left.isBusy() &&
            !Back_Right.isBusy()
          )
        ) {}
    }
  }

  public void RightStrafeToPosition(double distance, double mp) {
    int moveCounts = (int) (distance * COUNTS_PER_INCH);
    Front_Left.setPower(mp);
    Front_Right.setPower(-mp);
    Back_Left.setPower(-mp);
    Back_Right.setPower(mp);

    leftTarget = Front_Left.getCurrentPosition() + moveCounts;
    leftTarget = Back_Left.getCurrentPosition() + moveCounts;
    rightTarget = Front_Right.getCurrentPosition() + moveCounts;
    rightTarget = Back_Right.getCurrentPosition() + moveCounts;

    Front_Left.setTargetPosition(-leftTarget);
    Back_Right.setTargetPosition(-rightTarget);
    Back_Left.setTargetPosition(leftTarget);
    Front_Right.setTargetPosition(rightTarget);

    Front_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Front_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Back_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    Back_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    while (
      opModeIsActive() &&
      (
        Front_Left.isBusy() &&
        Front_Right.isBusy() &&
        Back_Left.isBusy() &&
        Back_Right.isBusy()
      )
    );
    moveRobot(0, 0);
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
    // int ecFR;
    // double inFR;
    // int ecPerInch;

    // ecFR = 637.304;
    // inFR = 12.75;
    // ecPerInch = ecFR / (int) inFR;

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

    Front_Left.setPower(leftSpeed);
    Back_Left.setPower(leftSpeed);
    Back_Right.setPower(rightSpeed);
    Front_Right.setPower(rightSpeed);
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

}
