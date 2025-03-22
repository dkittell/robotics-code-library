package org.firstinspires.ftc.teamcode.Teleop;

//region imports
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
import java.lang.Math;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

//endregion imports

@TeleOp(name = "Field Oriented Iterative 2023", group = "Iterative Opmode")
public class Scooter_FieldOriented_Iterative_2023 extends OpMode {

  //region Declaring Constants/Variables
  private CRServo sBackIntake;
  private CRServo sFrontIntake;
  private CRServo sLeftIntake1;
  private CRServo sLeftIntake2;
  private CRServo sRightIntake1;
  private CRServo sRightIntake2;
  //region Color Sensors
  // private ColorSensor csBackIntake;
  // private ColorSensor csFrontIntake;
  // private DistanceSensor cdsBackIntake;
  // private DistanceSensor cdsFrontIntake;
  //endregion Color Sensors
  private DcMotor mBackLeft = null;
  private DcMotor mBackRight = null;
  private DcMotor mFrontLeft = null;
  private DcMotor mFrontRight = null;
  private DcMotor mLinear1;
  private DcMotor mLinear2;
  private DcMotor mPivot;
  private ElapsedTime runtime = new ElapsedTime();
  private IMU imu;

  private static final boolean USE_WEBCAM = true; // true for webcam, false for phone camera
  private AprilTagProcessor aprilTag;
  private VisionPortal visionPortal;

  //endregion Declaring Constants/Variables

  @Override
  public void init() {
    //region Initialization
    DcMotor mBackLeft = hardwareMap.dcMotor.get("mBackLeft");
    DcMotor mBackRight = hardwareMap.dcMotor.get("mBackRight");
    DcMotor mFrontLeft = hardwareMap.dcMotor.get("mFrontLeft");
    DcMotor mFrontRight = hardwareMap.dcMotor.get("mFrontRight");
    IMU imu = hardwareMap.get(IMU.class, "imu");

    //region Color Sensors
    // cdsBackIntake = hardwareMap.get(DistanceSensor.class, "csBackIntake");
    // cdsFrontIntake = hardwareMap.get(DistanceSensor.class, "csFrontIntake");
    // csBackIntake = hardwareMap.get(ColorSensor.class, "csBackIntake");
    // csFrontIntake = hardwareMap.get(ColorSensor.class, "csFrontIntake");
    //endregion Color Sensors
    mLinear1 = hardwareMap.get(DcMotor.class, "mLinear1");
    mLinear2 = hardwareMap.get(DcMotor.class, "mLinear2");
    mPivot = hardwareMap.get(DcMotor.class, "mPivot");
    sBackIntake = hardwareMap.get(CRServo.class, "sBackIntake");
    sFrontIntake = hardwareMap.get(CRServo.class, "sFrontIntake");
    sLeftIntake1 = hardwareMap.get(CRServo.class, "sLeftIntake1");
    sLeftIntake2 = hardwareMap.get(CRServo.class, "sLeftIntake2");
    sRightIntake1 = hardwareMap.get(CRServo.class, "sRightIntake1");
    sRightIntake2 = hardwareMap.get(CRServo.class, "sRightIntake2");

    mBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    mFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    mLinear2.setDirection(DcMotor.Direction.REVERSE);
    mPivot.setDirection(DcMotor.Direction.REVERSE);

    mBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    mBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    mFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    mFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    mBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    mBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    mFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    mFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    mBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    mBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    mFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    mFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    mLinear1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    mLinear2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    mPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    initAprilTag();

    IMU.Parameters parameters = new IMU.Parameters(
      new RevHubOrientationOnRobot(
        RevHubOrientationOnRobot.LogoFacingDirection.UP,
        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
      )
    );
    imu.initialize(parameters);
    telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
    telemetry.addData(">", "Touch Play to start OpMode");
    telemetry.addData("Status", "Initialized");
    //endregion Initialization
  }

  @Override
  public void init_loop() {
    // Insert code here to initialize for better telop work
  }

  @Override
  public void start() {
    runtime.reset();
    boolean toggle = false;
  }

  @Override
  public void loop() {
    //region Color Sensors
    // NormalizedRGBA normalizedColors1;
    // NormalizedRGBA normalizedColors2;
    // float hue1 = 0.0f;
    // float hue2 = 0.0f;
    // float saturation1 = 0.0f;
    // float saturation2 = 0.0f;
    // float value1 = 0.0f;
    // float value2 = 0.0f;
    // int color1;
    // int color2;
    // int gain1 = 2;
    // int gain2 = 2;
    //endregion Color Sensors

    double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
    double x = gamepad1.left_stick_x;
    double rx = gamepad1.right_stick_x;

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
    double backLeftPower = (rotY - rotX + rx) / denominator;
    double backRightPower = (rotY + rotX - rx) / denominator;
    double frontLeftPower = (rotY + rotX + rx) / denominator;
    double frontRightPower = (rotY - rotX - rx) / denominator;
    //endregion Gyro

    mBackLeft.setPower(backLeftPower);
    mBackRight.setPower(backRightPower);
    mFrontLeft.setPower(frontLeftPower);
    mFrontRight.setPower(frontRightPower);

    if (gamepad2.left_stick_y > 0.17) {
      mLinear1.setPower(-0.7);
      mLinear2.setPower(-0.7);
    } else if (gamepad2.left_stick_y < -0.17) {
      mLinear1.setPower(0.7);
      mLinear2.setPower(0.7);
    } else {
      mLinear1.setPower(0);
      mLinear2.setPower(0);
    }
    if (gamepad2.right_stick_y > 0.17) {
      mPivot.setPower(1);
    } else if (gamepad2.right_stick_y < -0.17) {
      mPivot.setPower(-1);
    } else {
      mPivot.setPower(0);
    }

    if (gamepad2.left_trigger > 0.17) {
      sLeftIntake1.setPower(1);
      sLeftIntake2.setPower(1);
      sRightIntake1.setPower(1);
      sRightIntake2.setPower(1);
    } else if (gamepad2.right_trigger > 0.17) {
      sLeftIntake1.setPower(-1);
      sLeftIntake2.setPower(-1);
      sRightIntake1.setPower(-1);
      sRightIntake2.setPower(-1);
    } else {
      sLeftIntake1.setPower(0);
      sLeftIntake2.setPower(0);
      sRightIntake1.setPower(0);
      sRightIntake2.setPower(0);
    }

    telemetry.update();

    //region April Tags
    telemetryAprilTag();
    telemetry.update();

    // Save CPU resources; can resume streaming when needed.
    if (gamepad1.dpad_down) {
      visionPortal.stopStreaming();
    } else if (gamepad1.dpad_up) {
      visionPortal.resumeStreaming();
    }
    //endregion April Tags

    //region Color Sensor
    // telemetry.addData(
    //   "Dist to tgt (cm)",
    //   (cdsFrontIntake.getDistance(DistanceUnit.CM))
    // );
    // // Display reflected light.
    // telemetry.addData(
    //   "Light detected",
    //   ((OpticalDistanceSensor) cdsFrontIntake).getLightDetected()
    // );
    // // Adjust the gain.
    // if (gamepad1.a) {
    //   gain1 += 0.005;
    //   gain2 += 0.005;
    // } else if (gamepad1.b && gain1 >= 1.005) {
    //   gain1 += -0.005;
    //   gain2 += -0.005;
    // }
    // ((NormalizedColorSensor) csFrontIntake).setGain(gain1);
    // // telemetry.addData(
    // //   "Gain 1",
    // //   ((NormalizedColorSensor) csFrontIntake).getGain()
    // // );
    // // Read color from the sensor.
    // normalizedColors1 =
    //   ((NormalizedColorSensor) csFrontIntake).getNormalizedColors();

    // ((NormalizedColorSensor) csBackIntake).setGain(gain2);
    // // telemetry.addData(
    // //   "Gain 2",
    // //   ((NormalizedColorSensor) csBackIntake).getGain()
    // // );
    // // Read color from the sensor.
    // normalizedColors2 =
    //   ((NormalizedColorSensor) csBackIntake).getNormalizedColors();
    // // telemetry.addData("Red 1",
    // //   Double.parseDouble(JavaUtil.formatNumber(normalizedColors1.red, 3))
    // // );
    // // telemetry.addData(
    // //   "Green 1",
    // //   Double.parseDouble(JavaUtil.formatNumber(normalizedColors1.green, 3))
    // // );
    // // telemetry.addData("Blue 1",Double.parseDouble(JavaUtil.formatNumber(normalizedColors1.blue, 3)));

    // // telemetry.addData(
    // //   "Red 2",
    // //   Double.parseDouble(JavaUtil.formatNumber(normalizedColors2.red, 3))
    // // );
    // // telemetry.addData(
    // //   "Green 2",
    // //   Double.parseDouble(JavaUtil.formatNumber(normalizedColors2.green, 3))
    // // );
    // // telemetry.addData(
    // //   "Blue 2",
    // //   Double.parseDouble(JavaUtil.formatNumber(normalizedColors2.blue, 3))
    // // );

    // // Convert RGB values to Hue, Saturation, and Value.
    // // See https://en.wikipedia.org/wiki/HSL_and_HSV for details on HSV color model.
    // //region Front Color Sensor
    // color1 = normalizedColors1.toColor();
    // hue1 = JavaUtil.colorToHue(color1);
    // saturation1 = JavaUtil.colorToSaturation(color1);
    // value1 = JavaUtil.colorToValue(color1);

    // telemetry.addData(
    //   "Hue 1",
    //   Double.parseDouble(JavaUtil.formatNumber(hue1, 0))
    // );
    // telemetry.addData(
    //   "Saturation 1",
    //   Double.parseDouble(JavaUtil.formatNumber(saturation1, 3))
    // );
    // telemetry.addData(
    //   "Value 1",
    //   Double.parseDouble(JavaUtil.formatNumber(value1, 3))
    // );
    // // telemetry.addData(
    // //   "Alpha 1",
    // //   Double.parseDouble(JavaUtil.formatNumber(normalizedColors1.alpha, 3))
    // // );

    // // Show the color on the Robot Controller screen.
    // JavaUtil.showColor(hardwareMap.appContext, color1);
    // //endregion Front Color Sensor

    // //region Back Color Sensor
    // color2 = normalizedColors2.toColor();
    // hue2 = JavaUtil.colorToHue(color2);
    // saturation2 = JavaUtil.colorToSaturation(color2);
    // value2 = JavaUtil.colorToValue(color2);
    // telemetry.addData(
    //   "Hue 2",
    //   Double.parseDouble(JavaUtil.formatNumber(hue2, 0))
    // );
    // telemetry.addData(
    //   "Saturation 2",
    //   Double.parseDouble(JavaUtil.formatNumber(saturation2, 3))
    // );
    // // telemetry.addData(
    // //   "Value 2",
    // //   Double.parseDouble(JavaUtil.formatNumber(value2, 3))
    // // );
    // // telemetry.addData(
    // //   "Alpha 2",
    // //   Double.parseDouble(JavaUtil.formatNumber(normalizedColors2.alpha, 3))
    // // );
    // // Show the color on the Robot Controller screen.
    // JavaUtil.showColor(hardwareMap.appContext, color2);
    // //endregion Back Color Sensor

    // // Use hue to determine if it's red, green, blue, etc..
    // // If we pick up a color wit the color sensor we stop the servo associated with that color sensor

    // //region Use the Color to determine when the claw servo moves

    // //region Back Intake
    // if ((hue2 > 80) && (hue2 < 110)) {
    //   telemetry.addData("Color 2", "Yellow");

    //   //      sleep(5);
    //   sBackIntake.setPower(0);
    //   telemetry.update();
    //   // Color = Green
    // } else if ((hue2 > 125) && (hue2 < 150)) {
    //   telemetry.addData("Color 2", "Green");

    //   //      sleep(5);
    //   sBackIntake.setPower(0);
    //   telemetry.update();
    //   // Color = Purple
    // } else if ((hue2 > 210) && (hue2 < 230)) {
    //   telemetry.addData("Color 2", "Purple");

    //   //      sleep(5);
    //   sBackIntake.setPower(0);
    //   telemetry.update();
    // } else if (hue2 < 70) {
    //   telemetry.addData("Color 2", "No Color");

    //   sBackIntake.setPower(1);
    //   telemetry.update();
    // } else {
    //   telemetry.addData("Color 2", "Unknown");

    //   sFrontIntake.setPower(1);
    //   telemetry.update();
    // }
    // if (saturation2 > 0.2) {
    //   // Check to see if it might be black or white.
    //   // telemetry.addData("Check Sat 2", "Is surface white?");
    //   telemetry.addData("Color 2", "Black or White?");

    //   //      sleep(5);
    //   sBackIntake.setPower(0);
    //   telemetry.update();
    // }

    // telemetry.update();
    // //endregion Back Intake

    // //region Front Intake
    // if ((hue1 > 80) && (hue1 < 110)) {
    //   telemetry.addData("Color 1", "Yellow");
    //   //      sleep(5);
    //   sFrontIntake.setPower(0);
    //   telemetry.update();
    //   // Color = Green
    // } else if ((hue1 > 125) && (hue1 < 150)) {
    //   telemetry.addData("Color 1", "Green");
    //   //      sleep(5);
    //   sFrontIntake.setPower(0);
    //   telemetry.update();
    //   // Color = Purple
    // } else if ((hue1 > 210) && (hue1 < 230)) {
    //   telemetry.addData("Color 1", "Purple");
    //   //      sleep(5);
    //   sFrontIntake.setPower(0);
    //   telemetry.update();
    // } else if (hue1 < 70) {
    //   telemetry.addData("Color 1", "No Color");
    //   sFrontIntake.setPower(1);
    //   telemetry.update();
    // } else {
    //   telemetry.addData("Color 1", "Unknown");
    //   sFrontIntake.setPower(1);
    //   telemetry.update();
    // }
    // if (saturation1 > 0.2) {
    //   // Check to see if it might be black or white.
    //   // telemetry.addData("Check Sat 1", "Is surface white?");
    //   telemetry.addData("Color 1", "Black or White?");
    //   //      sleep(5);
    //   sFrontIntake.setPower(0);
    //   telemetry.update();
    // }
    // //endregion Front Intake
    //endregion Color Sensor

    telemetry.update();
    telemetry.addData("FL %", frontLeftPower);
    telemetry.addData("FR %", frontRightPower);
    telemetry.addData("BL %", backLeftPower);
    telemetry.addData("BR %", backRightPower);
    telemetry.update();
  }

  @Override
  public void stop() {}

  private void initAprilTag() {
    // Create the AprilTag processor.
    aprilTag =
      new AprilTagProcessor.Builder()
        // The following default settings are available to un-comment and edit as needed.
        //.setDrawAxes(false)
        //.setDrawCubeProjection(false)
        //.setDrawTagOutline(true)
        //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
        //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
        //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

        // == CAMERA CALIBRATION ==
        // If you do not manually specify calibration parameters, the SDK will attempt
        // to load a predefined calibration for your camera.
        //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
        // ... these parameters are fx, fy, cx, cy.

        .build();

    // Adjust Image Decimation to trade-off detection-range for detection-rate.
    // eg: Some typical detection data using a Logitech C920 WebCam
    // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
    // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
    // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
    // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
    // Note: Decimation can be changed on-the-fly to adapt during a match.
    //aprilTag.setDecimation(3);

    // Create the vision portal by using a builder.
    VisionPortal.Builder builder = new VisionPortal.Builder();

    // Set the camera (webcam vs. built-in RC phone camera).
    if (USE_WEBCAM) {
      builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
    } else {
      builder.setCamera(BuiltinCameraDirection.BACK);
    }

    // Choose a camera resolution. Not all cameras support all resolutions.
    //builder.setCameraResolution(new Size(640, 480));

    // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
    //builder.enableLiveView(true);

    // Set the stream format; MJPEG uses less bandwidth than default YUY2.
    //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

    // Choose whether or not LiveView stops if no processors are enabled.
    // If set "true", monitor shows solid orange screen if no processors enabled.
    // If set "false", monitor shows camera view without annotations.
    //builder.setAutoStopLiveView(false);

    // Set and enable the processor.
    builder.addProcessor(aprilTag);

    // Build the Vision Portal, using the above settings.
    visionPortal = builder.build();
    // Disable or re-enable the aprilTag processor at any time.
    //visionPortal.setProcessorEnabled(aprilTag, true);

  } // end method initAprilTag()

  /**
   * Add telemetry about AprilTag detections.
   */
  private void telemetryAprilTag() {
    List<AprilTagDetection> currentDetections = aprilTag.getDetections();
    telemetry.addData("# AprilTags Detected", currentDetections.size());

    // Step through the list of detections and display info for each one.
    for (AprilTagDetection detection : currentDetections) {
      if (detection.metadata != null) {
        telemetry.addLine(
          String.format(
            "\n==== (ID %d) %s",
            detection.id,
            detection.metadata.name
          )
        );
        telemetry.addLine(
          String.format(
            "XYZ %6.1f %6.1f %6.1f  (inch)",
            detection.ftcPose.x,
            detection.ftcPose.y,
            detection.ftcPose.z
          )
        );
        telemetry.addLine(
          String.format(
            "PRY %6.1f %6.1f %6.1f  (deg)",
            detection.ftcPose.pitch,
            detection.ftcPose.roll,
            detection.ftcPose.yaw
          )
        );
        telemetry.addLine(
          String.format(
            "RBE %6.1f %6.1f %6.1f  (inch, deg, deg)",
            detection.ftcPose.range,
            detection.ftcPose.bearing,
            detection.ftcPose.elevation
          )
        );
      } else {
        telemetry.addLine(
          String.format("\n==== (ID %d) Unknown", detection.id)
        );
        telemetry.addLine(
          String.format(
            "Center %6.0f %6.0f   (pixels)",
            detection.center.x,
            detection.center.y
          )
        );
      }
    } // end for() loop

    // Add "key" information to telemetry
    telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
    telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
    telemetry.addLine("RBE = Range, Bearing & Elevation");
  } // end method telemetryAprilTag()
}
