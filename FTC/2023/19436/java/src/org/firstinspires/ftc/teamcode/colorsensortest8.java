package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "colorsensortest8 (Blocks to Java)")
public class colorsensortest8 extends LinearOpMode {

  private ColorSensor colorSensorAsColorSensor;
  private DcMotor LeftDrive;
  private DcMotor RightDrive;

  String _7BcolorVariable_7D;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    int red;
    int green;
    int blue;

    colorSensorAsColorSensor = hardwareMap.get(ColorSensor.class, "colorSensorAsColorSensor");
    LeftDrive = hardwareMap.get(DcMotor.class, "LeftDrive");
    RightDrive = hardwareMap.get(DcMotor.class, "RightDrive");

    // Put initialization blocks here.
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        red = colorSensorAsColorSensor.red();
        green = colorSensorAsColorSensor.green();
        blue = colorSensorAsColorSensor.blue();
        _7BcolorVariable_7D = "notResponding";
        if (red > green && red > blue) {
          autonDrive1();
        } else if (green > red && green > blue) {
          autonDrive2();
          LeftDrive.setPower(1);
          RightDrive.setPower(1);
          LeftDrive.setTargetPosition(0);
          RightDrive.setTargetPosition(0);
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
    telemetry.addData("Red", colorSensorAsColorSensor.red());
    telemetry.addData("Green", colorSensorAsColorSensor.green());
    telemetry.addData("Blue", colorSensorAsColorSensor.blue());
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
