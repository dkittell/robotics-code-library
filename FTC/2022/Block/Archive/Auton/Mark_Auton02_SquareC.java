package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech;

@Autonomous(name = "Mark Auton 02 Square C (Java)", group = "")
public class Mark_Auton02_SquareC extends LinearOpMode {
  private AndroidTextToSpeech androidTextToSpeech;
  private DcMotor FrontRight;
  private DcMotor BackRight;
  private DcMotor FrontLeft;
  private DcMotor BackLeft;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    // region Set Variables

    // region Distance measurements to encoder ticks
    // 1 full rotation is 13 inches
    // 537.6 tics is 1 full rotation

    double tFullRotation = 537.6;
    double iFullRotation = 13.0; // inches
    double tPerInch = tFullRotation / iFullRotation;
    double dInchesToSquareC = 125;
    int tForwardToSquareC = (int) Math.round(tPerInch * dInchesToSquareC);
    double dStrafeRight01 = 22.0; // inches
    int tStrafeRight01 = (int) Math.round(tPerInch * dStrafeRight01);
    double dStrafeLeft01 = 25.0; // inches
    int tStrafeLeft01 = (int) Math.round(tPerInch * dStrafeLeft01);
    double dInchesBackwards01 = 50.0; // inches
    int tBackward01 = (int) Math.round(tPerInch * dInchesBackwards01);
    // endregion Distance measurements to encoder ticks

    // Motor Power Variable
    double mP = 0.5;

    androidTextToSpeech = new AndroidTextToSpeech();
    // endregion Set Variables

    // region Speak Initialization
    androidTextToSpeech.initialize();
    androidTextToSpeech.setLanguageAndCountry("en", "US");
    // endregion Speak Initialization

    // region Motor Initialization
    FrontRight = hardwareMap.get(DcMotor.class, "Front Right");
    BackRight = hardwareMap.get(DcMotor.class, "Back Right");
    FrontLeft = hardwareMap.get(DcMotor.class, "Front Left");
    BackLeft = hardwareMap.get(DcMotor.class, "Back Left");
    FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
    BackRight.setDirection(DcMotorSimple.Direction.REVERSE);
    // endregion Motor Initialization

    // region Encoder Initialization
    FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    // Move forward 39244.8 ticks 73 in
    FrontLeft.setTargetPosition(tForwardToSquareC);
    BackLeft.setTargetPosition(tForwardToSquareC);
    FrontRight.setTargetPosition(tForwardToSquareC);
    BackRight.setTargetPosition(tForwardToSquareC);
    FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    // endregion Encoder Initialization

    telemetry.addData("Action needed", "Please press the start triangle");
    telemetry.addData("Tics To Move", tForwardToSquareC);
    telemetry.update();
    waitForStart();

    // region move forward1
    // Run forward until all motors have achieved their ticks
    androidTextToSpeech.speak("I will move forward to square C");
    FrontLeft.setPower(mP);
    BackLeft.setPower(mP);
    FrontRight.setPower(mP);
    BackRight.setPower(mP);
    //endregion move forward1

    while (
      !(
        !FrontLeft.isBusy() &&
        !BackLeft.isBusy() &&
        !FrontRight.isBusy() &&
        !BackRight.isBusy()
      )
    ) {}

    // region reset encoder and reverse direction
    // Reset Encoders: All Motors
    // androidTextToSpeech.speak("Resetting Encoders");
    FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    // Right side motors: invert/reverse move command
    FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
    BackRight.setDirection(DcMotorSimple.Direction.REVERSE);
    // endregion reset encoder and reverese direction

    // region strafe right

    // Move Robot: Strafe right
    androidTextToSpeech.speak("Strafe Right");
    // Set TargetPosition and ticks
    FrontLeft.setTargetPosition(tStrafeRight01);
    BackLeft.setTargetPosition(-tStrafeRight01);
    FrontRight.setTargetPosition(-tStrafeLeft01);
    BackRight.setTargetPosition(tStrafeRight01);
    // Set Mode to RUN_TO_POSITION for all motors
    FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    // endregion strafe right

    while (
      !(
        !FrontLeft.isBusy() &&
        !BackLeft.isBusy() &&
        !FrontRight.isBusy() &&
        !BackRight.isBusy()
      )
    ) {}

    // Add code to drop wobble
    // While dropping wobble using servo do not move motors

    // Wait two seconds
    sleep(2000);

    // region reset encoder and reverse direction
    // Reset Encoders: All Motors
    // androidTextToSpeech.speak("Resetting Encoders");
    FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    // Right side motors: invert/reverse move command
    FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
    BackRight.setDirection(DcMotorSimple.Direction.REVERSE);
    // endregion reset encoder and reverese direction

    // region strafe left

    // Move Robot: Strafe Left
    androidTextToSpeech.speak("Strafe Left");
    // Set TargetPosition and ticks
    FrontLeft.setTargetPosition(-tStrafeLeft01);
    BackLeft.setTargetPosition(tStrafeLeft01);
    FrontRight.setTargetPosition(tStrafeLeft01);
    BackRight.setTargetPosition(-tStrafeLeft01);
    // Set Mode to RUN_TO_POSITION for all motors
    FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    // endregion strafe left

    while (
      !(
        !FrontLeft.isBusy() &&
        !BackLeft.isBusy() &&
        !FrontRight.isBusy() &&
        !BackRight.isBusy()
      )
    ) {}

    FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    // Right side motors: invert/reverse move command
    FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
    BackRight.setDirection(DcMotorSimple.Direction.REVERSE);
    // endregion reset encoder and reverese direction

    // region strafe left

    // Move Robot: Strafe Left
    androidTextToSpeech.speak("Move Backward");
    // Set TargetPosition and ticks
    FrontLeft.setTargetPosition(-tBackward01);
    BackLeft.setTargetPosition(-tBackward01);
    FrontRight.setTargetPosition(-tBackward01);
    BackRight.setTargetPosition(-tBackward01);
    // Set Mode to RUN_TO_POSITION for all motors
    FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    // endregion strafe left

    while (
      !(
        !FrontLeft.isBusy() &&
        !BackLeft.isBusy() &&
        !FrontRight.isBusy() &&
        !BackRight.isBusy()
      )
    ) {}

    // region reset encode and reverse direction
    // Reset encoders reverse motors right set
    //  ticks tell motors to move
    //
    FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
    BackRight.setDirection(DcMotorSimple.Direction.REVERSE);

    // endregion reset encode and reverse direction

    // All Motors

    androidTextToSpeech.speak("I am done");
    FrontRight.setPower(0);
    BackRight.setPower(0);
    FrontLeft.setPower(0);
    BackRight.setPower(0);

    while (
      !(
        !FrontLeft.isBusy() &&
        !BackLeft.isBusy() &&
        !FrontRight.isBusy() &&
        !BackRight.isBusy()
      )
    ) {}
  }
}
