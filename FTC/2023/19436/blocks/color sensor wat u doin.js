// IDENTIFIERS_USED=colorSensorAsColorSensor

/**
 * This function is executed when this Op Mode is selected from the Driver Station.
 */
function runOpMode() {
  linearOpMode.waitForStart();
  if (linearOpMode.opModeIsActive()) {
    while (linearOpMode.opModeIsActive()) {
      telemetry.addNumericData('red', colorSensorAsColorSensor.getRed());
      telemetry.addNumericData('green', colorSensorAsColorSensor.getGreen() * 0.8);
      telemetry.addNumericData('blue', colorSensorAsColorSensor.getBlue() + 0);
      telemetry.update();
    }
  }
}
