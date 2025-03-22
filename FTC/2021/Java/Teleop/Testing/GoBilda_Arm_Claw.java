package org.firstinspires.ftc.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp(name = "GoBilda_Arm_Claw (Blocks to Java)")
public class GoBilda_Arm_Claw extends LinearOpMode {

  private DcMotor m_Arm;
  private CRServo Grab_One;
  private CRServo Grab_Two;
  private CRServo Grab_Three;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    m_Arm = hardwareMap.get(DcMotor.class, "m_Arm");
    Grab_One = hardwareMap.get(CRServo.class, "Grab_One");
    Grab_Two = hardwareMap.get(CRServo.class, "Grab_Two");
    Grab_Three = hardwareMap.get(CRServo.class, "Grab_Three");
    double Gear2;
    int Dpaddownlast2;
    int Dpaduplast2;
    // Put initialization blocks here.
    m_Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    m_Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    m_Arm.setDirection(DcMotorSimple.Direction.REVERSE);
    m_Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    Gear2 = 0.1;
    Dpaddownlast2 = 0;
    Dpaduplast2 = 0;
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
          if (Dpaduplast2 == 0) {
          if (Gear2 < 0.79) {
            if (gamepad1.dpad_up) {
              Dpaduplast2 = 1;
              Gear2 += 0.1;
            }
          }
        } else if (!gamepad1.dpad_up) {
          Dpaduplast2 = 0;
        }
        if (Dpaddownlast2 == 0) {
          if (Gear2 > 0.11) {
            if (gamepad1.dpad_down) {
              Gear2 += -0.1;
              Dpaddownlast2 = 1;
            }
          }
        } else if (!gamepad1.dpad_down) {
          Dpaddownlast2 = 0;
        }
      m_Arm.setPower((gamepad1.left_stick_y * Gear2));
        telemetry.addData("Gear_Arm", Gear2);
         if (gamepad1.right_bumper) {
          Grab_One.setPower(1);
          Grab_Two.setPower(1);
          Grab_Three.setPower(1);
        } else if (gamepad1.left_bumper) {
          Grab_One.setPower(-1);
          Grab_Two.setPower(-1);
          Grab_Three.setPower(-1);
        } else {
          Grab_One.setPower(0);
          Grab_Two.setPower(0);
          Grab_Three.setPower(0);
        }
        
        telemetry.update();
      }
    }
  }
}
