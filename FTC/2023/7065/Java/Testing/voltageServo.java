// package org.firstinspires.ftc.teamcode.Testing;
// 
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.eventloop.opmode.Disabled;
// import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// import com.qualcomm.robotcore.hardware.CRServo;
// 
// @Disabled
// 
// @TeleOp(name = "voltageServo (Blocks to Java)")
// public class voltageServo extends LinearOpMode {
// 
//   private CRServo sBackLeftIntake;
//   private CRServo sBackRightIntake;
//   private CRServo sFrontLeftIntake;
//   private CRServo sFrontRightIntake;
// 
//   /**
//    * This function is executed when this OpMode is selected from the Driver Station.
//    */
//   @Override
//   public void runOpMode() {
//     double sBackLeftPower;
//     double sBackRightPower;
//     double sFrontLeftPower;
//     double sFrontRightPower;
// 
//     sBackLeftIntake = hardwareMap.get(CRServo.class, "sBackLeftIntake");
//     sBackRightIntake = hardwareMap.get(CRServo.class, "sBackRightIntake");
//     sFrontLeftIntake = hardwareMap.get(CRServo.class, "sFrontLeftIntake");
//     sFrontRightIntake = hardwareMap.get(CRServo.class, "sFrontRightIntake");
// 
//     // Put initialization blocks here.
//     sBackLeftPower = sBackLeftIntake.getPower();
//     sBackRightPower = sBackRightIntake.getPower();
//     sFrontLeftPower = sFrontLeftIntake.getPower();
//     sFrontRightPower = sFrontRightIntake.getPower();
//     waitForStart();
//     if (opModeIsActive()) {
//       // Put run blocks here.
//       while (opModeIsActive()) {
//           if (gamepad2.left_trigger > 0.17) {
//           sFrontLeftIntake.setPower(1);
//           sBackLeftIntake.setPower(1);
//           sFrontRightIntake.setPower(1);
//           sBackRightIntake.setPower(1);
//         } else if (gamepad2.right_trigger > 0.17) {
//           sFrontLeftIntake.setPower(-1);
//           sBackLeftIntake.setPower(-1);
//           sFrontRightIntake.setPower(-1);
//           sBackRightIntake.setPower(-1);
//         } else {
//           sFrontLeftIntake.setPower(0);
//           sBackLeftIntake.setPower(0);
//           sFrontRightIntake.setPower(0);
//           sBackRightIntake.setPower(0);
//         }
//         // Put loop blocks here.
//         telemetry.update();
//       }
//         telemetry.update();
//         telemetry.addData("sBackLeftPower", sBackLeftPower);
//         telemetry.addData("sBackRightPower", sBackRightPower);
//         telemetry.addData("sFrontLeftPower", sFrontLeftPower);
//         telemetry.addData("sFrontRightPower", sFrontRightPower);
//     }
//   }
// }
// 
