package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "colorsensortest (Blocks to Java)")
public class colorsensortest extends LinearOpMode {

  private ColorSensor colorSensor;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    int _7BcolorVariable_7D;

    colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

    // Put initialization blocks here.
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        _7BcolorVariable_7D = Color.rgb(colorSensor.red(), colorSensor.green(), colorSensor.blue());
        telemetry.addData("Color", JavaUtil.colorToHue(_7BcolorVariable_7D));
        telemetry.update();
      }
    }
  }
}
