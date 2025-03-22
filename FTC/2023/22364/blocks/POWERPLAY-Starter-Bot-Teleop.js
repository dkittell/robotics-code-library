// IDENTIFIERS_USED=gamepad1,intakeservoAsCRServo,left_driveAsDcMotor,leftarmmotorAsDcMotor,right_driveAsDcMotor,rightarmmotorAsDcMotor

var turnSpeed, driveSpeed, armSpeed;

/**
 * This function is executed when this Op Mode is selected from the Driver Station.
 */
function runOpMode() {
  // You will have to determine which motor to reverse for your robot.
  // In this example, the right motor was reversed, so that positive
  // applied power makes it move the robot in the forward direction.
  leftarmmotorAsDcMotor.setDirection("REVERSE");
  left_driveAsDcMotor.setDirection("REVERSE");
  turnSpeed = 0.6;
  driveSpeed = 0.8;
  armSpeed = -0.8;
  linearOpMode.waitForStart();
  if (linearOpMode.opModeIsActive()) {
    while (linearOpMode.opModeIsActive()) {
      // The Y axis of a joystick ranges from -1 in its topmost
      // position to +1 in the bottom position. We negate (i.e.
      // reverse the positive/negative sign of) these values, so that
      // pushing the joystick up will create maximum forward power.
      left_driveAsDcMotor.setDualPower(-(gamepad1.getLeftStickY() * driveSpeed) + gamepad1.getRightStickX() * turnSpeed, right_driveAsDcMotor, -(gamepad1.getLeftStickY() * driveSpeed) - gamepad1.getRightStickX() * turnSpeed);
      if (gamepad1.getDpadDown()) {
        leftarmmotorAsDcMotor.setDualPower(0.1, rightarmmotorAsDcMotor, 0.1);
      } else {
        if (gamepad1.getDpadUp()) {
          leftarmmotorAsDcMotor.setPower(armSpeed);
          rightarmmotorAsDcMotor.setPower(armSpeed);
        } else {
          leftarmmotorAsDcMotor.setPower(-0.15);
          rightarmmotorAsDcMotor.setPower(-0.15);
        }
      }
      if (gamepad1.getRightBumper()) {
        intakeservoAsCRServo.setPower(-1);
      } else if (gamepad1.getLeftBumper()) {
        intakeservoAsCRServo.setPower(0.8);
      } else {
        intakeservoAsCRServo.setPower(0.6);
      }
      telemetry.addNumericData('Left Pow', right_driveAsDcMotor.getPower());
      telemetry.addNumericData('Right Pow', left_driveAsDcMotor.getPower());
      telemetry.update();
      telemetry.addNumericData('driveSpeed', driveSpeed);
      telemetry.addNumericData('turnSpeed', turnSpeed);
    }
  }
}
