package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "POWERPLAYStarterBotTeleop4 (Blocks to Java)")
public class POWERPLAYStarterBotTeleop4 extends LinearOpMode {

  private DcMotor leftarmmotor;
  private DcMotor right_drive;
  private DcMotor left_drive;
  private DcMotor rightarmmotor;
  private CRServo intakeservo;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    double turnSpeed;
    double armSpeed;
    double driveSpeed;

    leftarmmotor = hardwareMap.get(DcMotor.class, "left arm motor");
    right_drive = hardwareMap.get(DcMotor.class, "right_drive");
    left_drive = hardwareMap.get(DcMotor.class, "left_drive");
    rightarmmotor = hardwareMap.get(DcMotor.class, "right arm motor");
    intakeservo = hardwareMap.get(CRServo.class, "intake servo");

    // You will need to reverse one of the drive motors.
    // You will have to determine which motor to reverse for your robot.
    // In this example, the right motor was reversed, so that positive
    // applied power makes it move the robot in the forward direction.
    leftarmmotor.setDirection(DcMotorSimple.Direction.REVERSE);
    right_drive.setDirection(DcMotorSimple.Direction.REVERSE);
    turnSpeed = 0.6;
    turnSpeed = 0.8;
    armSpeed = 0.5;
    waitForStart();
    if (opModeIsActive()) {
      // Put your run blocks here.
      while (opModeIsActive()) {
        // Put your loop blocks here.
        // Use left stick to drive and right stick to turn
        // The Y axis of a joystick ranges from -1 in its topmost
        // position to +1 in the bottom position. We negate (i.e.
        // reverse the positive/negative sign of) these values, so that
        // pushing the joystick up will create maximum forward power.
        left_drive.setPower(-(gamepad1.left_stick_y * driveSpeed) - gamepad1.right_stick_x * turnSpeed);
        right_drive.setPower(-(gamepad1.left_stick_y * driveSpeed) + gamepad1.right_stick_x * turnSpeed);
        // If DPadUp is pressed, lift raises.
        if (gamepad1.dpad_down) {
          leftarmmotor.setPower(-armSpeed);
          rightarmmotor.setPower(-armSpeed);
        } else {
          // If DPadDown is pressed, lift lowers.
          if (gamepad1.dpad_up) {
            leftarmmotor.setPower(armSpeed);
            rightarmmotor.setPower(armSpeed);
          } else {
            // If DPad is not pressed, lift remains still.
            leftarmmotor.setPower(0);
            rightarmmotor.setPower(0);
          }
        }
        // If RightBumper is pressed, Intake receives power
        if (gamepad1.right_bumper) {
          intakeservo.setPower(-1);
        } else {
          intakeservo.setPower(0.3);
        }
        // Code below details data collection during OpMode
        telemetry.addData("Left Pow", right_drive.getPower());
        telemetry.addData("Right Pow", left_drive.getPower());
        telemetry.update();
      }
    }
  }
}
