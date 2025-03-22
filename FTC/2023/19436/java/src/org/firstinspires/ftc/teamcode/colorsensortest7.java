package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "colorsensortest7 (Blocks to Java)")
public class colorsensortest7 extends LinearOpMode {

  private ColorSensor colorSensor;

  String _7BcolorVariable_7D;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    int red;
    int green;
    int blue;

    colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

    // Put initialization blocks here.
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        red = colorSensor.red();
        green = colorSensor.green();
        blue = colorSensor.blue();
        _7BcolorVariable_7D = "notResponding";
        if (red > green && red > blue) {
          autonDrive1();
        } else if (green > red && green > blue) {
          autonDrive2();
        } else if (blue > green && blue > red) {
          autonDrive3();
        }
        updateColorTelemetry();
        telemetry.update();
      }
    }
  }

  /**
   * Describe this function...
   */
  private void updateColorTelemetry() {
    telemetry.addData("Color", _7BcolorVariable_7D);
    telemetry.addData("Red", colorSensor.red());
    telemetry.addData("Green", colorSensor.green());
    telemetry.addData("Blue", colorSensor.blue());
  }

  /**
   * Describe this function...
   */
  private void autonDrive1() {
    _7BcolorVariable_7D = "red";
  }

  /**
   * Describe this function...
   */
  private void autonDrive2() {
    _7BcolorVariable_7D = "green";
  }

  /**
   * Describe this function...
   */
  private void autonDrive3() {
    _7BcolorVariable_7D = "blue";
  }
}
