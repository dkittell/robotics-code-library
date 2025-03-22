package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "teachinglogan (Blocks to Java)")
public class teachinglogan extends LinearOpMode {

  private DcMotor leftarmmotor;
  private DcMotor left_drive;
  private DcMotor right_drive;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    leftarmmotor = hardwareMap.get(DcMotor.class, "left arm motor");
    left_drive = hardwareMap.get(DcMotor.class, "left_drive");
    right_drive = hardwareMap.get(DcMotor.class, "right_drive");

    // Put initialization blocks here.
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        telemetry.update();
        // You will have to determine which motor to reverse for your robot.
        // In this example, the right motor was reversed, so that positive
        // applied power makes it move the robot in the forward direction.
        leftarmmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        left_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        left_drive.setPower(3);
        right_drive.setPower(2);
        sleep(2000);
        leftarmmotor.setPower(0);
        left_drive.setPower(0);
      }
    }
  }
}
