package org.firstinspires.ftc.teamcode.Teleop;

//region Imports
import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//endregion Imports

 @Disabled
@TeleOp(name = "Scooter Drivetrain 2023", group = "Iterative Opmode")
public class Scooter_Drivetrain_2023 extends OpMode {

  // Declare OpMode members.
  private ElapsedTime runtime = new ElapsedTime();

  private DcMotor mBackLeft = null;
  private DcMotor mFrontLeft = null;
  private DcMotor mFrontRight = null;
  private DcMotor mBackRight = null;

  private CRServo sFrontIntake;
  private CRServo sBackIntake;
  private ColorSensor csFrontIntake;
  private ColorSensor csBackIntake;
  // private ColorSensor csMiddleIntake;
  private DistanceSensor cdsFrontIntake;
  private DistanceSensor cdsBackIntake;
  // private DistanceSensor cdsMiddleIntake;

  public double dDriverGear;
  public int nDriverDpadUpLast;
  public int nDriverDpadDownLast;
  public int eU;
  public double inchL;
  public double mP;
  public double inchM;
  public double inchH;
  public double inchG;
  public boolean toggle;
  public String sColor2 = "";
  public boolean bPixelDetected1 = false;
  public boolean bPixelDetected2 = false;
  int ecHomeLinear;
  int ecHighPivot;
  int ecLowPivot;
  int ecHomePivot;
  int ecLowLinear;
  int ecHighLinear;
  int ecClimbPivot;
  double mpLinear;
  double mpPivot;

  /*
   * Code to run ONCE when the driver hits INIT
   */
  @Override
  public void init() {
    // Initialize the hardware variables. Note that the strings used here as parameters
    // to 'get' must correspond to the names assigned during the robot configuration
    // step (using the FTC Robot Controller app on the phone).

    cdsFrontIntake = hardwareMap.get(DistanceSensor.class, "csFrontIntake");
    cdsBackIntake = hardwareMap.get(DistanceSensor.class, "csBackIntake");
    // cdsMiddleIntake = hardwareMap.get(DistanceSensor.class, "csMiddleIntake");
    csFrontIntake = hardwareMap.get(ColorSensor.class, "csFrontIntake");
    csBackIntake = hardwareMap.get(ColorSensor.class, "csBackIntake");
    // csMiddleIntake = hardwareMap.get(ColorSensor.class, "csMiddleIntake");
    sFrontIntake = hardwareMap.get(CRServo.class, "sFrontIntake");
    sBackIntake = hardwareMap.get(CRServo.class, "sBackIntake");

    mBackLeft = hardwareMap.get(DcMotor.class, "mBackLeft");
    mFrontLeft = hardwareMap.get(DcMotor.class, "mFrontLeft");
    mFrontRight = hardwareMap.get(DcMotor.class, "mFrontRight");
    mBackRight = hardwareMap.get(DcMotor.class, "mBackRight");

    // Most robots need the motor on one side to be reversed to drive forward
    // Reverse the motor that runs backwards when connected directly to the battery

    mBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    mFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    mFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
    mBackRight.setDirection(DcMotorSimple.Direction.FORWARD);

    mFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    mFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    mBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    mBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    mFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    mFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    mBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    mBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    mFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    mBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    mFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    mBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    // region Set values to variables
    dDriverGear = 0.8;
    nDriverDpadUpLast = 0; // Last known value for dpadUp - Driver Controller
    nDriverDpadDownLast = 0; // Last known value for dpadDown - Driver Controller

    eU = 637; // Encoder count for full rotation

    // 637 x 1.9 = 13.75 in
    inchL = eU * 2.2;
    mP = 1.0;
    // 637 x 3.3 = 23.75 in
    inchM = eU * 3.5;
    // 637 x 4.8 = 34.03125 in
    inchH = eU * 4.6;
    // 637 x 0.3 =
    inchG = eU * 0.7;
    
    // int nColorSensor = 1;
    //   for (int i = 1; i <= nColorSensors; i++) {
    //     PixelColorSensor(i);
    //   }

    // endregion Set values to variables

    // Tell the driver that initialization is complete.
    telemetry.addData("Status", "Initialized");
  }

  /*
   * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
   */
  @Override
  public void init_loop() {
    // Insert code here to initialize for better telop work
  }

  /*
   * Code to run ONCE when the driver hits PLAY
   */
  @Override
  public void start() {
    runtime.reset();
    boolean toggle = false;
  }

  /*
   * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
   */
  @Override
  public void loop() {
    NormalizedRGBA normalizedColors1;
    NormalizedRGBA normalizedColors2;
    float hue1 = 0.0f;
    float hue2 = 0.0f;
    float saturation1 = 0.0f;
    float saturation2 = 0.0f;
    float value1 = 0.0f;
    float value2 = 0.0f;
    int color1;
    int color2;
    int gain1 = 2;
    int gain2 = 2;

    //region Drive Code
    // Setup a variable for each drive wheel to save power level for telemetry
    // region Drive Code (up and down dpad)
    if (nDriverDpadUpLast == 0) {
      if (dDriverGear < 0.89) {
        if (gamepad1.dpad_up) {
          nDriverDpadUpLast = 1;
          dDriverGear += 0.1;
        }
      }
    } else if (!gamepad1.dpad_up) {
      nDriverDpadUpLast = 0;
    }
    if (nDriverDpadDownLast == 0) {
      if (dDriverGear > 0.11) {
        if (gamepad1.dpad_down) {
          dDriverGear += -0.1;
          nDriverDpadDownLast = 1;
        }
      }
    } else if (!gamepad1.dpad_down) {
      nDriverDpadDownLast = 0;
    }
    // endregion Drive Code (up and down dpad)

    // region Driver Controller

    // region Simply Drive Robot
    mFrontLeft.setPower(
      -gamepad1.left_stick_y *
      dDriverGear +
      gamepad1.right_stick_x *
      dDriverGear +
      gamepad1.left_stick_x *
      dDriverGear
    );
    mFrontRight.setPower(
      (
        -gamepad1.left_stick_y *
        dDriverGear -
        gamepad1.right_stick_x *
        dDriverGear
      ) -
      gamepad1.left_stick_x *
      dDriverGear
    );

    mBackLeft.setPower(
      (
        -gamepad1.left_stick_y *
        dDriverGear +
        gamepad1.right_stick_x *
        dDriverGear
      ) -
      gamepad1.left_stick_x *
      dDriverGear
    );

    mBackRight.setPower(
      (
        -gamepad1.left_stick_y *
        dDriverGear -
        gamepad1.right_stick_x *
        dDriverGear
      ) +
      gamepad1.left_stick_x *
      dDriverGear
    );
    // endregion Simply Drive Robot
    // endregion Driver Controller
    
     
      

   

    telemetry.addData("Time Passed", getRuntime());
    telemetry.addData("Gear_Drive", dDriverGear);
    //endregion Drive Code

    //region Color Sensor Code
    // Display distance info.
    telemetry.addData(
      "Dist to tgt (cm)",
      (cdsFrontIntake.getDistance(DistanceUnit.CM))
    );
    // Display reflected light.
    telemetry.addData(
      "Light detected",
      ((OpticalDistanceSensor) cdsFrontIntake).getLightDetected()
    );
    // Adjust the gain.
    if (gamepad1.a) {
      gain1 += 0.005;
      gain2 += 0.005;
    } else if (gamepad1.b && gain1 >= 1.005) {
      gain1 += -0.005;
      gain2 += -0.005;
    }
    ((NormalizedColorSensor) csFrontIntake).setGain(gain1);
    // telemetry.addData(
    //   "Gain 1",
    //   ((NormalizedColorSensor) csFrontIntake).getGain()
    // );
    // Read color from the sensor.
    normalizedColors1 =
      ((NormalizedColorSensor) csFrontIntake).getNormalizedColors();

    ((NormalizedColorSensor) csBackIntake).setGain(gain2);
    // telemetry.addData(
    //   "Gain 2",
    //   ((NormalizedColorSensor) csBackIntake).getGain()
    // );
    // Read color from the sensor.
    normalizedColors2 =
      ((NormalizedColorSensor) csBackIntake).getNormalizedColors();
    // telemetry.addData("Red 1",
    //   Double.parseDouble(JavaUtil.formatNumber(normalizedColors1.red, 3))
    // );
    // telemetry.addData(
    //   "Green 1",
    //   Double.parseDouble(JavaUtil.formatNumber(normalizedColors1.green, 3))
    // );
    // telemetry.addData("Blue 1",Double.parseDouble(JavaUtil.formatNumber(normalizedColors1.blue, 3)));

    // telemetry.addData(
    //   "Red 2",
    //   Double.parseDouble(JavaUtil.formatNumber(normalizedColors2.red, 3))
    // );
    // telemetry.addData(
    //   "Green 2",
    //   Double.parseDouble(JavaUtil.formatNumber(normalizedColors2.green, 3))
    // );
    // telemetry.addData(
    //   "Blue 2",
    //   Double.parseDouble(JavaUtil.formatNumber(normalizedColors2.blue, 3))
    // );

    // Convert RGB values to Hue, Saturation, and Value.
    // See https://en.wikipedia.org/wiki/HSL_and_HSV for details on HSV color model.
    //region Front Color Sensor
    color1 = normalizedColors1.toColor();
    hue1 = JavaUtil.colorToHue(color1);
    saturation1 = JavaUtil.colorToSaturation(color1);
    value1 = JavaUtil.colorToValue(color1);

    telemetry.addData(
      "Hue 1",
      Double.parseDouble(JavaUtil.formatNumber(hue1, 0))
    );
    telemetry.addData(
      "Saturation 1",
      Double.parseDouble(JavaUtil.formatNumber(saturation1, 3))
    );
    telemetry.addData(
      "Value 1",
      Double.parseDouble(JavaUtil.formatNumber(value1, 3))
    );
    // telemetry.addData(
    //   "Alpha 1",
    //   Double.parseDouble(JavaUtil.formatNumber(normalizedColors1.alpha, 3))
    // );

    // Show the color on the Robot Controller screen.
    JavaUtil.showColor(hardwareMap.appContext, color1);
    //endregion Front Color Sensor

    //region Back Color Sensor
    color2 = normalizedColors2.toColor();
    hue2 = JavaUtil.colorToHue(color2);
    saturation2 = JavaUtil.colorToSaturation(color2);
    value2 = JavaUtil.colorToValue(color2);
    telemetry.addData(
      "Hue 2",
      Double.parseDouble(JavaUtil.formatNumber(hue2, 0))
    );
    telemetry.addData(
      "Saturation 2",
      Double.parseDouble(JavaUtil.formatNumber(saturation2, 3))
    );
    // telemetry.addData(
    //   "Value 2",
    //   Double.parseDouble(JavaUtil.formatNumber(value2, 3))
    // );
    // telemetry.addData(
    //   "Alpha 2",
    //   Double.parseDouble(JavaUtil.formatNumber(normalizedColors2.alpha, 3))
    // );
    // Show the color on the Robot Controller screen.
    JavaUtil.showColor(hardwareMap.appContext, color2);
    //endregion Back Color Sensor

    // Use hue to determine if it's red, green, blue, etc..
    // If we pick up a color wit the color sensor we stop the servo associated with that color sensor

    //region Use the Color to determine when the claw servo moves

    //region Back Intake
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
    // telemetry.update();
    //endregion Use the Color to determine when the claw servo moves

    //endregion Color Sensor Code

  }

  /*
   * Code to run ONCE after the driver hits STOP
   */
  @Override
  public void stop() {}
  
  // public void PixelColorSensor(int nColorSensor) {
  //   NormalizedRGBA normalizedColors;
  //   String csName = "";

  //   switch (nColorSensor) {
  //     case 1:
  //       // Use Back Color Sensor
  //       cdsPixel = hardwareMap.get(ColorSensor.class, "csBackIntake");
  //       csName = "Back Color Sensor";
  //       break;
  //     case 2:
  //       // Use Front Color Sensor
  //       cdsPixel = hardwareMap.get(ColorSensor.class, "csFrontIntake");
  //       csName = "Front Color Sensor";
  //       break;
  //     default:
  //       // Use Back Color Sensor
  //       cdsPixel = hardwareMap.get(ColorSensor.class, "csBackIntake");
  //       csName = "Back Color Sensor";
  //       break;
  //   }

  //   JavaUtil.showColor(hardwareMap.appContext, nColor);
  //   // Display distance info.
  //   // telemetry.addData(
  //   //   "Dist to tgt (cm)",
  //   //   ((DistanceSensor) cdsPixel).getDistance(
  //   //       DistanceUnit.CM
  //   //     )
  //   // );
  //   // Display reflected light.
  //   // telemetry.addData(
  //   //   "Light detected",
  //   //   (
  //   //     (OpticalDistanceSensor) cdsPixel
  //   //   ).getLightDetected()
  //   // );

  //   ((NormalizedColorSensor) cdsPixel).setGain(fGain);
  //   // telemetry.addData(
  //   //   "Gain",
  //   //   ((NormalizedColorSensor) cdsPixel).getGain()
  //   // );
  //   // Read color from the sensor.
  //   normalizedColors = ((NormalizedColorSensor) cdsPixel).getNormalizedColors();

  //   nColor = normalizedColors.toColor();
  //   fHue = JavaUtil.colorToHue(nColor);
  //   fSaturation = JavaUtil.colorToSaturation(nColor);
  //   fValue = JavaUtil.colorToValue(nColor);
  //   telemetry.addData(
  //     csName + " " + "Hue",
  //     Double.parseDouble(JavaUtil.formatNumber(fHue, 0))
  //   );
  //   // telemetry.addData(
  //   //   "Saturation",
  //   //   Double.parseDouble(JavaUtil.formatNumber(fSaturation, 3))
  //   // );
  //   // Show the color on the Robot Controller screen.
  //   // Use hue to determine if it's red, green, blue, etc..
  //   if (fHue >= 80 && fHue <= 90) {
  //     dDistance = ((DistanceSensor) cdsPixel).getDistance(DistanceUnit.CM);
  //     sColor = "Yellow";
  //     bPixelDetected = true;
  //   } else if (fHue >= 120 && fHue <= 134) {
  //     dDistance = ((DistanceSensor) cdsPixel).getDistance(DistanceUnit.CM);
  //     sColor = "Green";
  //     bPixelDetected = true;
  //   } else if (fHue >= 135 && fHue <= 150) {
  //     dDistance = ((DistanceSensor) cdsPixel).getDistance(DistanceUnit.CM);
  //     sColor = "White";
  //     bPixelDetected = true;
  //   } else if (fHue >= 200 && fHue <= 210) {
  //     dDistance = ((DistanceSensor) cdsPixel).getDistance(DistanceUnit.CM);
  //     sColor = "Purple";
  //     bPixelDetected = true;
  //   } else if (
  //     ((DistanceSensor) cdsPixel).getDistance(DistanceUnit.CM) >= 0.600 &&
  //     ((DistanceSensor) cdsPixel).getDistance(DistanceUnit.CM) <= 0.636
  //   ) {
  //     dDistance = ((DistanceSensor) cdsPixel).getDistance(DistanceUnit.CM);
  //     sColor = "Unknown";
  //     bPixelDetected = true;
  //   } else {
  //     telemetry.addData(csName + " " + "Color", "Unknown");
  //     bPixelDetected = false;
  //   }
  //   // Check to see if it might be black or white.
  //   if (fSaturation < 0.2) {
  //     telemetry.addData(csName + " " + "Check Sat", "Is surface white?");
  //     bPixelDetected = false;
  //   }
  //   telemetry.update();
  //   if (fValue < 0.16) {
  //     telemetry.addData(csName + " " + "Check Val", "Is surface black?");
  //     bPixelDetected = false;
  //   }

  //   if (nColorSensor == 1) {
  //     // Use Back Color Sensor
  //     sColor1 = sColor;
  //     bPixelDetected1 = bPixelDetected;
  //   } else if (nColorSensor == 2) {
  //     // Use Front Color Sensor
  //     sColor2 = sColor;
  //     bPixelDetected2 = bPixelDetected;
  //   } else {
  //     bPixelDetected1 = false;
  //     bPixelDetected2 = false;
  //   }

  //   telemetry.addData(csName + " " + "Color", sColor);
  //   telemetry.addData(csName + " " + "Pixel Detected", bPixelDetected);
  //   telemetry.update();
  }

