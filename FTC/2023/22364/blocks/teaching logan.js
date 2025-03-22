// IDENTIFIERS_USED=gamepad1,left_driveAsDcMotor,leftarmmotorAsDcMotor,right_driveAsDcMotor

var turnSpeed, driveSpeed;

/**
 * This function is executed when this Op Mode is selected from the Driver Station.
 */
function runOpMode() {
  linearOpMode.waitForStart();
  if (linearOpMode.opModeIsActive()) {
    while (linearOpMode.opModeIsActive()) {
      telemetry.update();
      // You will have to determine which motor to reverse for your robot.
      // In this example, the right motor was reversed, so that positive
      // applied power makes it move the robot in the forward direction.
      leftarmmotorAsDcMotor.setDirection("REVERSE");
      left_driveAsDcMotor.setDirection("REVERSE");
      // The Y axis of a joystick ranges from -1 in its topmost
      // position to +1 in the bottom position. We negate (i.e.
      // reverse the positive/negative sign of) these values, so that
      // pushing the joystick up will create maximum forward power.
      left_driveAsDcMotor.setDualPower(-(gamepad1.getLeftStickY() * driveSpeed) + gamepad1.getRightStickX() * turnSpeed, right_driveAsDcMotor, -(gamepad1.getLeftStickY() * driveSpeed) - gamepad1.getRightStickX() * turnSpeed);
      left_driveAsDcMotor.setDualPower(3, right_driveAsDcMotor, 2);
      linearOpMode.sleep(2000);
      leftarmmotorAsDcMotor.setDualPower(0, left_driveAsDcMotor, 0);
    }
  }
}
