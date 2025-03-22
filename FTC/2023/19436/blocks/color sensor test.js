// IDENTIFIERS_USED=ArmAsDcMotor,colorSensorAsColorSensor,IntakeAsDcMotor,LeftDriveAsDcMotor,RightDriveAsDcMotor

var red, green, blue;

/**
 * Describe this function...
 */
function autonDrive2() {
  LeftDriveAsDcMotor.setDualPower(-0.25, RightDriveAsDcMotor, -0.25);
  linearOpMode.sleep(2000);
  LeftDriveAsDcMotor.setDualPower(0, RightDriveAsDcMotor, 0);
}

/**
 * Describe this function...
 */
function autonDrive1() {
  LeftDriveAsDcMotor.setDualPower(-0.2, RightDriveAsDcMotor, -0.2);
  linearOpMode.sleep(1000);
  LeftDriveAsDcMotor.setDualPower(0.3, RightDriveAsDcMotor, -0.3);
  linearOpMode.sleep(2000);
  LeftDriveAsDcMotor.setDualPower(-0.47, RightDriveAsDcMotor, -0.47);
  linearOpMode.sleep(1700);
  LeftDriveAsDcMotor.setDualPower(0, RightDriveAsDcMotor, 0);
}

/**
 * This function is executed when this Op Mode is selected from the Driver Station.
 */
function runOpMode() {
  RightDriveAsDcMotor.setDirection("REVERSE");
  linearOpMode.waitForStart();
  LeftDriveAsDcMotor.setDualPower(-0.3, RightDriveAsDcMotor, -0.3);
  linearOpMode.sleep(1700);
  LeftDriveAsDcMotor.setDualPower(0, RightDriveAsDcMotor, 0);
  ArmAsDcMotor.setDualPower(0.25, IntakeAsDcMotor, 0.5);
  linearOpMode.sleep(1000);
  ArmAsDcMotor.setDualPower(0, IntakeAsDcMotor, 0);
  red = colorSensorAsColorSensor.getRed();
  green = colorSensorAsColorSensor.getGreen() * 0.8;
  blue = colorSensorAsColorSensor.getBlue();
  telemetry.addNumericData('red', red);
  telemetry.addNumericData('green', green);
  telemetry.addNumericData('blue', blue);
  if (red > green && red > blue) {
    autonDrive1();
  } else if (green > red && green > blue) {
    autonDrive2();
  } else if (blue > green && blue > red) {
    autonDrive3();
  }
  telemetry.update();
  if (linearOpMode.opModeIsActive()) {
    while (linearOpMode.opModeIsActive()) {
    }
  }
}

/**
 * Describe this function...
 */
function autonDrive3() {
  LeftDriveAsDcMotor.setDualPower(-0.2, RightDriveAsDcMotor, -0.2);
  linearOpMode.sleep(800);
  LeftDriveAsDcMotor.setDualPower(-0.3, RightDriveAsDcMotor, 0.3);
  linearOpMode.sleep(2400);
  LeftDriveAsDcMotor.setDualPower(-0.47, RightDriveAsDcMotor, -0.47);
  linearOpMode.sleep(1750);
  LeftDriveAsDcMotor.setDualPower(0, RightDriveAsDcMotor, 0);
}
