// package org.firstinspires.ftc.teamcode.Teleop;
// 
// import android.graphics.Color;
// import com.qualcomm.robotcore.eventloop.opmode.Disabled;
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.hardware.CRServo;
// import com.qualcomm.robotcore.hardware.ColorSensor;
// import com.qualcomm.robotcore.hardware.DcMotor;
// import com.qualcomm.robotcore.hardware.DistanceSensor;
// import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
// import com.qualcomm.robotcore.hardware.NormalizedRGBA;
// import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
// import org.firstinspires.ftc.robotcore.external.JavaUtil;
// import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
// 
// @Disabled
// @TeleOp(name = "ColorSensorTest")
// public class ColorSensorTest extends LinearOpMode {
// 
//   private CRServo sFrontIntake;
//   private CRServo sBackIntake;
//   private ColorSensor csFrontIntake;
//   private ColorSensor csBackIntake;
//   // private ColorSensor csMiddleIntake;
//   private DistanceSensor cdsFrontIntake;
//   private DistanceSensor cdsBackIntake;
//   // private DistanceSensor cdsMiddleIntake;
// 
//   /**
//    * This function is executed when this OpMode is selected from the Driver Station.
//    */
//   @Override
//   public void runOpMode() {
//     NormalizedRGBA normalizedColors1;
//     NormalizedRGBA normalizedColors2;
//     float hue1 = 0.0f;
//     float hue2 = 0.0f;
//     float saturation1 = 0.0f;
//     float saturation2 = 0.0f;
//     float value1 = 0.0f;
//     float value2 = 0.0f;
//     int color1;
//     int color2;
//     int gain1 = 2;
//     int gain2 = 2;
// 
//     cdsFrontIntake = hardwareMap.get(DistanceSensor.class, "csFrontIntake");
//     cdsBackIntake = hardwareMap.get(DistanceSensor.class, "csBackIntake");
//     // cdsMiddleIntake = hardwareMap.get(DistanceSensor.class, "csMiddleIntake");
//     csFrontIntake = hardwareMap.get(ColorSensor.class, "csFrontIntake");
//     csBackIntake = hardwareMap.get(ColorSensor.class, "csBackIntake");
//     // csMiddleIntake = hardwareMap.get(ColorSensor.class, "csMiddleIntake");
//     sFrontIntake = hardwareMap.get(CRServo.class, "sFrontIntake");
//     sBackIntake = hardwareMap.get(CRServo.class, "sBackIntake");
// 
//     telemetry.addData("Color Distance Example", "Press start to continue...");
//     telemetry.update();
//     waitForStart();
//     if (opModeIsActive()) {
//       // Put run blocks here.
//       while (opModeIsActive()) {
//        
// 
//         // Display distance info.
//         telemetry.addData(
//           "Dist to tgt (cm)",
//           (cdsFrontIntake.getDistance(DistanceUnit.CM))
//         );
//         // Display reflected light.
//         telemetry.addData(
//           "Light detected",
//           ((OpticalDistanceSensor) cdsFrontIntake).getLightDetected()
//         );
//         // Adjust the gain.
//         if (gamepad1.a) {
//           gain1 += 0.005;
//           gain2 += 0.005;
//         } else if (gamepad1.b && gain1 >= 1.005) {
//           gain1 += -0.005;
//           gain2 += -0.005;
//         }
//         ((NormalizedColorSensor) csFrontIntake).setGain(gain1);
//         // telemetry.addData(
//         //   "Gain 1",
//         //   ((NormalizedColorSensor) csFrontIntake).getGain()
//         // );
//         // Read color from the sensor.
//         normalizedColors1 =
//           ((NormalizedColorSensor) csFrontIntake).getNormalizedColors();
// 
//         ((NormalizedColorSensor) csBackIntake).setGain(gain2);
//         // telemetry.addData(
//         //   "Gain 2",
//         //   ((NormalizedColorSensor) csBackIntake).getGain()
//         // );
//         // Read color from the sensor.
//         normalizedColors2 =
//           ((NormalizedColorSensor) csBackIntake).getNormalizedColors();
//         // telemetry.addData("Red 1",
//         //   Double.parseDouble(JavaUtil.formatNumber(normalizedColors1.red, 3))
//         // );
//         // telemetry.addData(
//         //   "Green 1",
//         //   Double.parseDouble(JavaUtil.formatNumber(normalizedColors1.green, 3))
//         // );
//         // telemetry.addData("Blue 1",Double.parseDouble(JavaUtil.formatNumber(normalizedColors1.blue, 3)));
// 
//         // telemetry.addData(
//         //   "Red 2",
//         //   Double.parseDouble(JavaUtil.formatNumber(normalizedColors2.red, 3))
//         // );
//         // telemetry.addData(
//         //   "Green 2",
//         //   Double.parseDouble(JavaUtil.formatNumber(normalizedColors2.green, 3))
//         // );
//         // telemetry.addData(
//         //   "Blue 2",
//         //   Double.parseDouble(JavaUtil.formatNumber(normalizedColors2.blue, 3))
//         // );
// 
//         // Convert RGB values to Hue, Saturation, and Value.
//         // See https://en.wikipedia.org/wiki/HSL_and_HSV for details on HSV color model.
//         //region Front Color Sensor
//         color1 = normalizedColors1.toColor();
//         hue1 = JavaUtil.colorToHue(color1);
//         saturation1 = JavaUtil.colorToSaturation(color1);
//         value1 = JavaUtil.colorToValue(color1);
// 
//         telemetry.addData(
//           "Hue 1",
//           Double.parseDouble(JavaUtil.formatNumber(hue1, 0))
//         );
//         telemetry.addData(
//           "Saturation 1",
//           Double.parseDouble(JavaUtil.formatNumber(saturation1, 3))
//         );
//         telemetry.addData(
//           "Value 1",
//           Double.parseDouble(JavaUtil.formatNumber(value1, 3))
//         );
//         // telemetry.addData(
//         //   "Alpha 1",
//         //   Double.parseDouble(JavaUtil.formatNumber(normalizedColors1.alpha, 3))
//         // );
// 
//         // Show the color on the Robot Controller screen.
//         JavaUtil.showColor(hardwareMap.appContext, color1);
//         //endregion Front Color Sensor
// 
//         //region Back Color Sensor
//         color2 = normalizedColors2.toColor();
//         hue2 = JavaUtil.colorToHue(color2);
//         saturation2 = JavaUtil.colorToSaturation(color2);
//         value2 = JavaUtil.colorToValue(color2);
//         telemetry.addData(
//           "Hue 2",
//           Double.parseDouble(JavaUtil.formatNumber(hue2, 0))
//         );
//         telemetry.addData(
//           "Saturation 2",
//           Double.parseDouble(JavaUtil.formatNumber(saturation2, 3))
//         );
//         // telemetry.addData(
//         //   "Value 2",
//         //   Double.parseDouble(JavaUtil.formatNumber(value2, 3))
//         // );
//         // telemetry.addData(
//         //   "Alpha 2",
//         //   Double.parseDouble(JavaUtil.formatNumber(normalizedColors2.alpha, 3))
//         // );
//         // Show the color on the Robot Controller screen.
//         JavaUtil.showColor(hardwareMap.appContext, color2);
//         //endregion Back Color Sensor
// 
//         // Use hue to determine if it's red, green, blue, etc..
//         // If we pick up a color wit the color sensor we stop the servo associated with that color sensor
// 
//         //region Use the Color to determine when the claw servo moves
// 
//         //region Back Intake
//         if ((hue2 > 80) && (hue2 < 110)) {
//           telemetry.addData("Color 2", "Yellow");
// 
//           sleep(5);
//           sBackIntake.setPower(0);
//           telemetry.update();
//           // Color = Green
//         } else if ((hue2 > 125) && (hue2 < 150)) {
//           telemetry.addData("Color 2", "Green");
// 
//           sleep(5);
//           sBackIntake.setPower(0);
//           telemetry.update();
//           // Color = Purple
//         } else if ((hue2 > 210) && (hue2 < 230)) {
//           telemetry.addData("Color 2", "Purple");
// 
//           sleep(5);
//           sBackIntake.setPower(0);
//           telemetry.update();
//         } else if (hue2 < 70) {
//           telemetry.addData("Color 2", "No Color");
// 
//           sBackIntake.setPower(1);
//           telemetry.update();
//         } else {
//           telemetry.addData("Color 2", "Unknown");
// 
//           sFrontIntake.setPower(1);
//           telemetry.update();
//         }
//         if (saturation2 > 0.2) {
//           // Check to see if it might be black or white.
//           // telemetry.addData("Check Sat 2", "Is surface white?");
//           telemetry.addData("Color 2", "Black or White?");
// 
//           sleep(5);
//           sBackIntake.setPower(0);
//           telemetry.update();
//         }
// 
//         telemetry.update();
//         //endregion Back Intake
// 
//         //region Front Intake
//         if ((hue1 > 80) && (hue1 < 110)) {
//           telemetry.addData("Color 1", "Yellow");
//           sleep(5);
//           sFrontIntake.setPower(0);
//           telemetry.update();
//           // Color = Green
//         } else if ((hue1 > 125) && (hue1 < 150)) {
//           telemetry.addData("Color 1", "Green");
//           sleep(5);
//           sFrontIntake.setPower(0);
//           telemetry.update();
//           // Color = Purple
//         } else if ((hue1 > 210) && (hue1 < 230)) {
//           telemetry.addData("Color 1", "Purple");
//           sleep(5);
//           sFrontIntake.setPower(0);
//           telemetry.update();
//         } else if (hue1 < 70) {
//           telemetry.addData("Color 1", "No Color");
//           sFrontIntake.setPower(1);
//           telemetry.update();
//         } else {
//           telemetry.addData("Color 1", "Unknown");
//           sFrontIntake.setPower(1);
//           telemetry.update();
//         }
//         if (saturation1 > 0.2) {
//           // Check to see if it might be black or white.
//           // telemetry.addData("Check Sat 1", "Is surface white?");
//           telemetry.addData("Color 1", "Black or White?");
//           sleep(5);
//           sFrontIntake.setPower(0);
//           telemetry.update();
//         }
//         //endregion Front Intake
//         telemetry.update();
//       }
//       // sFrontIntake.setPower(0);
//       // sBackIntake.setPower(0);
//       //endregion Use the Color to determine when the claw servo moves
//     }
// 
//     // Show white on the Robot Controller screen.
//     JavaUtil.showColor(hardwareMap.appContext, Color.parseColor("white"));
//   }
// }
// 
