package org.firstinspires.ftc.Testing;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@Autonomous(name = "Touch Color Code")
public class Touch_Color_Code extends LinearOpMode {

  private DcMotor Back_Left;
  private DcMotor Front_Left;
  private ColorSensor Color_Sensor_REV_ColorRangeSensor;
  private DcMotor Back_Right;
  private DcMotor Front_Right;
  private ColorSensor Color_Sensor;
  private DistanceSensor Color_Sensor_DistanceSensor;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    int CurrentColor;

    Back_Left = hardwareMap.get(DcMotor.class, "Back_Left");
    Front_Left = hardwareMap.get(DcMotor.class, "Front_Left");
    Color_Sensor_REV_ColorRangeSensor = hardwareMap.get(ColorSensor.class, "Color_Sensor");
    Back_Right = hardwareMap.get(DcMotor.class, "Back_Right");
    Front_Right = hardwareMap.get(DcMotor.class, "Front_Right");
    Color_Sensor = hardwareMap.get(ColorSensor.class, "Color_Sensor");
    Color_Sensor_DistanceSensor = hardwareMap.get(DistanceSensor.class, "Color_Sensor");

    // Put initialization blocks here.
    Back_Left.setDirection(DcMotorSimple.Direction.REVERSE);
    Front_Left.setDirection(DcMotorSimple.Direction.REVERSE);
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        CurrentColor = Color.rgb(Color_Sensor_REV_ColorRangeSensor.red(), Color_Sensor_REV_ColorRangeSensor.green(), Color_Sensor_REV_ColorRangeSensor.blue());
        if (JavaUtil.colorToSaturation(CurrentColor) >= 0.6 && JavaUtil.colorToHue(CurrentColor) > 81 && JavaUtil.colorToHue(CurrentColor) < 140) {
          Back_Left.setPower(-0.3);
          Back_Right.setPower(-0.3);
          Front_Right.setPower(-0.3);
          Front_Left.setPower(-0.3);
        } else {
          Back_Left.setPower(0.5);
          Back_Right.setPower(0.5);
          Front_Left.setPower(0.5);
          Front_Right.setPower(0.5);
        }
        Color_Sensor.enableLed(true);
        telemetry.addData("Green", Color_Sensor_REV_ColorRangeSensor.green());
        telemetry.addData("Blue", Color_Sensor_REV_ColorRangeSensor.red());
        telemetry.addData("Red", Color_Sensor_REV_ColorRangeSensor.red());
        telemetry.addData("Distance", Color_Sensor_DistanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.update();
      }
    }
  }
}
