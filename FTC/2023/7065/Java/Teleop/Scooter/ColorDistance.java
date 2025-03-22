package org.firstinspires.ftc.teamcode;

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

@TeleOp(name = "ColorDistance", group = "Diagnostic")
public class ColorDistance extends LinearOpMode {

  //region Initial Constants
  private CRServo sBackLeftIntake;
  private CRServo sBackRightIntake;
  private CRServo sFrontLeftIntake;
  private CRServo sFrontRightIntake;
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
  public String sColor1 = "";
  public String sColor2 = "";
  public boolean bPixelDetected1 = false;
  public boolean bPixelDetected2 = false;

  //endregion Temporary Solution to get values from each color sensor

  //endregion Public Variables

  /**
   * This function is executed when this OpMode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    sBackLeftIntake = hardwareMap.get(CRServo.class, "sBackLeftIntake");
    sBackRightIntake = hardwareMap.get(CRServo.class, "sBackRightIntake");
    sFrontLeftIntake = hardwareMap.get(CRServo.class, "sFrontLeftIntake");
    sFrontRightIntake = hardwareMap.get(CRServo.class, "sFrontRightIntake");
    sFrontLeftIntake.setDirection(CRServo.Direction.REVERSE);
    sFrontRightIntake.setDirection(CRServo.Direction.REVERSE);
    // This OpMode demonstrates the color and distance features of the REV sensor.

    fGain = 2;

    telemetry.addData("Color Distance Example", "Press start to continue...");
    telemetry.update();
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
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

      if (gamepad2.left_bumper) {
        //region Move Motors based on color sensor readout
        // If back color sensor detects a pixel, stop back motors and start detecting a pixel only for front color sensor.
        // If front color sensor and back color sensor detect a pixel stop both front and back motors
        if (bPixelDetected1) {
          sBackLeftIntake.setPower(0);
          sBackRightIntake.setPower(0);
          if (bPixelDetected2) {
            sFrontLeftIntake.setPower(0);
            sFrontRightIntake.setPower(0);
          } else {
            sFrontLeftIntake.setPower(1);
            sFrontRightIntake.setPower(1);
          }
        } else {
          sBackLeftIntake.setPower(1);
          sBackRightIntake.setPower(1);
          sFrontLeftIntake.setPower(1);
          sFrontRightIntake.setPower(1);
        }
        //endregion Move Motors based on color sensor readout
      }

      }
      // Show white on the Robot Controller screen.
      JavaUtil.showColor(hardwareMap.appContext, Color.parseColor("white"));
    }
  }

  //region Functions

  public void PixelColorSensor(int nColorSensor) {
    NormalizedRGBA normalizedColors;
    String csName = "";

    switch (nColorSensor) {
      case 1:
        // Use Back Color Sensor
        cdsPixel = hardwareMap.get(ColorSensor.class, "csBackIntake");
        csName = "Back Color Sensor";
        break;
      case 2:
        // Use Front Color Sensor
        cdsPixel = hardwareMap.get(ColorSensor.class, "csFrontIntake");
        csName = "Front Color Sensor";
        break;
      default:
        // Use Back Color Sensor
        cdsPixel = hardwareMap.get(ColorSensor.class, "csBackIntake");
        csName = "Back Color Sensor";
        break;
    }

    JavaUtil.showColor(hardwareMap.appContext, nColor);
    // Display distance info.
    telemetry.addData(
      "Dist to tgt (cm)",
      ((DistanceSensor) cdsPixel).getDistance(
          DistanceUnit.CM
        )
    );
    // Display reflected light.
    telemetry.addData(
      "Light detected",
      (
        (OpticalDistanceSensor) cdsPixel
      ).getLightDetected()
    );

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
    if (fHue >= 80 && fHue <= 90) {
      dDistance = ((DistanceSensor) cdsPixel).getDistance(DistanceUnit.CM);
      sColor = "Yellow";
      bPixelDetected = true;
    } else if (fHue >= 120 && fHue <= 128) {
      dDistance = ((DistanceSensor) cdsPixel).getDistance(DistanceUnit.CM);
      sColor = "Green";
      bPixelDetected = true;
    } else if (fHue >= 135 && fHue <= 150) {
      dDistance = ((DistanceSensor) cdsPixel).getDistance(DistanceUnit.CM);
      sColor = "White";
      bPixelDetected = true;
    } else if (fHue >= 200 && fHue <= 210) {
      dDistance = ((DistanceSensor) cdsPixel).getDistance(DistanceUnit.CM);
      sColor = "Purple";
      bPixelDetected = true;
    } else if (
      ((DistanceSensor) cdsPixel).getDistance(DistanceUnit.CM) >= 0.600 &&
      ((DistanceSensor) cdsPixel).getDistance(DistanceUnit.CM) <= 0.636
    ) {
      dDistance = ((DistanceSensor) cdsPixel).getDistance(DistanceUnit.CM);
      sColor = "Unknown";
      bPixelDetected = true;
    } else {
      telemetry.addData(csName + " " + "Color", "Unknown");
      bPixelDetected = false;
    }
    // Check to see if it might be black or white.
    if (fSaturation < 0.2) {
      telemetry.addData(csName + " " + "Check Sat", "Is surface white?");
      bPixelDetected = false;
    }
    telemetry.update();
    if (fValue < 0.16) {
      telemetry.addData(csName + " " + "Check Val", "Is surface black?");
      bPixelDetected = false;
    }

    if (nColorSensor == 1) {
      // Use Back Color Sensor
      sColor1 = sColor;
      bPixelDetected1 = bPixelDetected;
    } else if (nColorSensor == 2) {
      // Use Front Color Sensor
      sColor2 = sColor;
      bPixelDetected2 = bPixelDetected;
    } else {
      bPixelDetected1 = false;
      bPixelDetected2 = false;
    }

    telemetry.addData(csName + " " + "Color", sColor);
    telemetry.addData(csName + " " + "Pixel Detected", bPixelDetected);
    telemetry.update();
  }
  //endregion Functions

}
