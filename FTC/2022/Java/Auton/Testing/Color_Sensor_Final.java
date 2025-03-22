package org.firstinspires.ftc.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@Autonomous(name = "Color_Sensor")
public class Color_Sensor_Final extends LinearOpMode {

  private DcMotor Back_Left;
  private DcMotor Front_Left;
  private ColorSensor Color_Sensor_REV_ColorRangeSensor;
  private ColorSensor Color_Sensor;
  private DcMotor Back_Right;
  private DcMotor Front_Right;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    int Green;
    double mP;
    int Red;
    int ms;
    int Blue;
    String CurrentColor;

    Back_Left = hardwareMap.get(DcMotor.class, "Back_Left");
    Front_Left = hardwareMap.get(DcMotor.class, "Front_Left");
    Color_Sensor_REV_ColorRangeSensor = hardwareMap.get(ColorSensor.class, "Color_Sensor");
    Color_Sensor = hardwareMap.get(ColorSensor.class, "Color_Sensor");
    Back_Right = hardwareMap.get(DcMotor.class, "Back_Right");
    Front_Right = hardwareMap.get(DcMotor.class, "Front_Right");

    Back_Left.setDirection(DcMotorSimple.Direction.REVERSE);
    Front_Left.setDirection(DcMotorSimple.Direction.REVERSE);
    Green = Color_Sensor_REV_ColorRangeSensor.green();
    Red = Color_Sensor_REV_ColorRangeSensor.red();
    Blue = Color_Sensor_REV_ColorRangeSensor.blue();
    mP = 0.5;
    ms = 3000;
    // Put initialization blocks here.
    waitForStart();
        Green = Color_Sensor_REV_ColorRangeSensor.green();
    Red = Color_Sensor_REV_ColorRangeSensor.red();
    Blue = Color_Sensor_REV_ColorRangeSensor.blue();
    if (opModeIsActive()) {
        Green = Color_Sensor_REV_ColorRangeSensor.green();
    Red = Color_Sensor_REV_ColorRangeSensor.red();
    Blue = Color_Sensor_REV_ColorRangeSensor.blue();
    //  sleep(ms);
      if (Red >= 22) {
        CurrentColor = "red";
      } else if (Green >= 22) {
        CurrentColor = "green";
      } else {
        CurrentColor = "other";
      }
      
      sleep(ms);
      telemetry.update();
      telemetry.addData("Color", CurrentColor);
      telemetry.addData("Red", Color_Sensor.red());
      telemetry.addData("Green", Color_Sensor.green());
      telemetry.addData("Blue", Color_Sensor.blue());
      telemetry.update();
      sleep(5000);
    }
  }
}
