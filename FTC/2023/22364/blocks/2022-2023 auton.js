// IDENTIFIERS_USED=intakeservoAsCRServo,left_driveAsDcMotor,leftarmmotorAsDcMotor,right_driveAsDcMotor,rightarmmotorAsDcMotor

var lDist, rDist, driveCountsPermm;

/**
 * This function is executed when this Op Mode is selected from the Driver Station.
 */
function runOpMode() {
  left_driveAsDcMotor.setDirection("REVERSE");
  leftarmmotorAsDcMotor.setDirection("REVERSE");
  driveCountsPermm = (20.1524 * 28) / (90 * Math.PI);
  linearOpMode.waitForStart();
  intakeservoAsCRServo.setPower(0.3);
  goToPositionmm(-700, -700);
  if (linearOpMode.opModeIsActive()) {
    while (linearOpMode.opModeIsActive()) {
      telemetry.update();
    }
  }
  leftarmmotorAsDcMotor.setDualPower(0.1, rightarmmotorAsDcMotor, 0.1);
}

/**
 * Describe this function...
 */
function goToPositionmm(lDist, rDist) {
  right_driveAsDcMotor.setDualTargetPosition(right_driveAsDcMotor.getCurrentPosition() + rDist * driveCountsPermm, left_driveAsDcMotor, left_driveAsDcMotor.getCurrentPosition() + lDist * driveCountsPermm);
  right_driveAsDcMotor.setDualMode("RUN_TO_POSITION", left_driveAsDcMotor, "RUN_TO_POSITION");
  right_driveAsDcMotor.setDualPower(-0.25, left_driveAsDcMotor, -0.25);
  while (right_driveAsDcMotor.isBusy() || left_driveAsDcMotor.isBusy()) {
  }
  right_driveAsDcMotor.setDualPower(0, left_driveAsDcMotor, 0);
}
