package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "POVDriveTest (Blocks to Java)")
public class POVDriveTest extends LinearOpMode {

  private DcMotor right_driveAsDcMotor;
  private DcMotor left_driveAsDcMotor;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    right_driveAsDcMotor = hardwareMap.get(DcMotor.class, "right_driveAsDcMotor");
    left_driveAsDcMotor = hardwareMap.get(DcMotor.class, "left_driveAsDcMotor");

    // Reverse one of the drive motors.
    // You will have to determine which motor to reverse for your robot.
    // In this example, the right motor was reversed so that positive
    // applied power makes it move the robot in the forward direction.
    right_driveAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        // Use left stick to drive and right stick to turn
        // The Y axis of a joystick ranges from -1 in its topmost position
        // to +1 in its bottommost position. We negate this value so that
        // the topmost position corresponds to maximum forward power.
        left_driveAsDcMotor.setPower(-gamepad1.left_stick_y + gamepad1.right_stick_x);
        right_driveAsDcMotor.setPower(-gamepad1.left_stick_y - gamepad1.right_stick_x);
        telemetry.addData("Left Pow", left_driveAsDcMotor.getPower());
        telemetry.addData("Right Pow", right_driveAsDcMotor.getPower());
        telemetry.update();
      }
    }
  }
}
