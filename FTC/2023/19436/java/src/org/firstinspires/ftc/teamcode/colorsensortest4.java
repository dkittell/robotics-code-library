package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "colorsensortest4 (Blocks to Java)")
public class colorsensortest4 extends LinearOpMode {

  private ColorSensor colorSensor;

  int _7BcolorVariable_7D;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    int blue;
    // TODO: Enter the type for variable named red
    UNKNOWN_TYPE red;
    // TODO: Enter the type for variable named green
    UNKNOWN_TYPE green;

    colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

    // Put initialization blocks here.
    blue = colorSensor.blue();
    if (red > green && red > blue) {
      autonDrive1();
    } else if (false) {
    } else {
    }
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        _7BcolorVariable_7D = Color.rgb(colorSensor.red(), colorSensor.green(), colorSensor.blue());
        // Put loop blocks here.
        updateColorTelemetry();
        telemetry.update();
      }
    }
  }

  /**
   * Describe this function...
   */
  private void autonDrive1() {
  }

  /**
   * Describe this function...
   */
  private void updateColorTelemetry() {
    telemetry.addData("Color", JavaUtil.colorToHue(_7BcolorVariable_7D));
    telemetry.addData("Red", colorSensor.red());
    telemetry.addData("Green", colorSensor.green());
    telemetry.addData("Blue", colorSensor.blue());
  }
}
