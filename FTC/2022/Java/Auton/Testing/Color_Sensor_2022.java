package org.firstinspires.ftc.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@Disabled
// i dont have the onbot java stuff yet
@Autonomous(name = "Color_Sensor_2022 (Blocks to Java)")
public class Color_Sensor_2022 extends LinearOpMode {

  private DcMotor Back_Left;
  private ColorSensor Color_Sensor;
  private DcMotor Front_Left;
  private DcMotor Front_Right;
  private DcMotor Back_Right;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  
  public void runOpMode() {
    PIDFCoefficients _7BpidfCoefficientsVariable_7D;
    float CurrentColor;

    
    
    Back_Left = hardwareMap.get(DcMotor.class, "Back_Left");
    Front_Left = hardwareMap.get(DcMotor.class, "Front_Left");
    Front_Right = hardwareMap.get(DcMotor.class, "Front_Right");
    Back_Right = hardwareMap.get(DcMotor.class, "Back_Right");
    Color_Sensor = hardwareMap.get(ColorSensor.class, "Color_Sensor");
    //Put initialization blocks here.
    Color_Sensor.enableLed(true);
    
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Current color int here do not change.
        CurrentColor = JavaUtil.rgbToHue(Color_Sensor.red(), Color_Sensor.green(), Color_Sensor.blue());

        /**This is the area the color hue is configured DONT CHANGE THE NUMBERS!!! because these numbers are very delicit
         * and are what makes the color censor run. This is ran on linear op mode (I think) but no matter what dont change this
         * variables whithout asking the person who last change the numbers to make sure its ok this did take 4hrs to figure out
         * so please dont break it...**/

        // color pink
          if (JavaUtil.colorToSaturation((int) CurrentColor) <= 0.4 && (JavaUtil.colorToHue((int) CurrentColor) >= 351 || JavaUtil.colorToHue((int) CurrentColor) <= 360)) {
            //add driving code here
          
          
        }
              // color purple
                if (JavaUtil.colorToSaturation((int) CurrentColor) <= 0.4 && (JavaUtil.colorToHue((int) CurrentColor) >= 317 || JavaUtil.colorToHue((int) CurrentColor) <= 270)) {
                  //add driving code here
                    Front_Left.setPower(.3);
                      Back_Right.setPower(.3);
                        sleep(5000);
                        }
                          else{// color blue
              if (JavaUtil.colorToSaturation((int) CurrentColor) <= 0.4 && (JavaUtil.colorToHue((int) CurrentColor) >= 269 || JavaUtil.colorToHue((int) CurrentColor) <= 202)) {
            //add driving code here

          }
            else{// color cyan
                if (JavaUtil.colorToSaturation((int) CurrentColor) <= 0.4 && (JavaUtil.colorToHue((int) CurrentColor) <= 201 || JavaUtil.colorToHue((int) CurrentColor) <= 154)) {
                  //add driving code here
          
          
                  }else{ // color green
                    if (JavaUtil.colorToSaturation((int) CurrentColor) == 0.4 && (JavaUtil.colorToHue((int) CurrentColor) >= 153 || JavaUtil.colorToHue((int) CurrentColor) <= 78)) {
                      //add driving code here
                        Front_Right.setPower(.3);
                        sleep(5000);
          
          
                          }else{// color yellow
                            if (JavaUtil.colorToSaturation((int) CurrentColor) <= 0.4 && (JavaUtil.colorToHue((int) CurrentColor) >= 77 || JavaUtil.colorToHue((int) CurrentColor) <= 38)) {
                              //add driving code here
                              Back_Left.setPower(.3);
                              sleep(5000);
                                }else{// color orange
                                  if (JavaUtil.colorToSaturation((int) CurrentColor) <= 0.4 && (JavaUtil.colorToHue((int) CurrentColor) >= 37 || JavaUtil.colorToHue((int) CurrentColor) <= 15)) {
                                    //add driving code here

                                      }else{
                                        // color red
                                          if (JavaUtil.colorToSaturation((int) CurrentColor) <= 0.4 && (JavaUtil.colorToHue((int) CurrentColor) >= 14 || JavaUtil.colorToHue((int) CurrentColor) <= 0)) {
                                            //add driving code here

        }
        }
          
        }
          
          
        }
          
        
          
        }
        
          
         
        
        
        
        

        }
       
        }
        
        
        
        telemetry.update();
      }
    }
  }
}
