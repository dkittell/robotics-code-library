package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech;

@Autonomous(name = "Mark Auton 01 Square B (Java)", group = "")
public class Mark_Auton01_SquareB extends LinearOpMode {
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
    double dInchesStrafeRightToSquareB = 120;
    int tRightStrafeToSquareB = (int) Math.round(tPerInch * dInchesStrafeRightToSquareB);
    double dInchesForward01 = 19.0; //inches 
    int tForward01 = (int) Math.round(tPerInch * dInchesForward01);
    double dInchesForward02 = 35.0; // inches
    int tForward02 = (int) Math.round(tPerInch * dInchesForward02);
    double dInchesBackward01 = 19.0; //inches 
    int tBackward01 = (int) Math.round(tPerInch * dInchesBackward01);
    double dStrafeLeftToPark = 35; // inches
    int tStrafeLeftToPark = (int) Math.round(tPerInch * dStrafeLeftToPark);
    // endregion Distance measurements to encoder ticks

    // Motor Power Variable
    double mP = 0.5;

    // Driving Encoder Ticks
    // tRS - Ticks Right Strafe
    // int tRS = tRightStrafeToSquareB;
    // tStrafeLeftToPark - Ticks Left Strafe To Park 
    // int tStrafeLeftToPark = 5376;
    // tF - Ticks Forward
    // int tF = tForward02
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

<<<<<<< Updated upstream
    // Move forward 10214.4 ticks 19 in
=======
    // Move forward 3000 ticks 79.5 in
>>>>>>> Stashed changes
    FrontLeft.setTargetPosition(tForward01);
    BackLeft.setTargetPosition(tForward01);
    FrontRight.setTargetPosition(tForward01);
    BackRight.setTargetPosition(tForward01);
    FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    // endregion Encoder Initialization

    telemetry.addData("Action needed", "Please press the start triangle");
    telemetry.addData("Tics To Move", tForward01);
    telemetry.update();
    waitForStart();

    // region move forward1
    // Run forward until all motors have achieved their ticks
    androidTextToSpeech.speak("I will move forward");
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
    // region move backward1
    // Reset Encoders: All Motors
    FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
<<<<<<< Updated upstream
    // Right side motors invert/reverse
    FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
    BackRight.setDirection(DcMotorSimple.Direction.REVERSE);
    // Make Robot tell you what it is doing
    androidTextToSpeech.speak("I will move backward");
    // Move Robot: Backward
=======

    // Move forward 3000 ticks 79.5 in
>>>>>>> Stashed changes
    FrontLeft.setTargetPosition(-tBackward01);
    BackLeft.setTargetPosition(-tBackward01);
    FrontRight.setTargetPosition(-tBackward01);
    BackRight.setTargetPosition(-tBackward01);
    FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
<<<<<<< Updated upstream
    // endregion move backward1
=======

    telemetry.addData("Front Left Encoder", FrontLeft.getCurrentPosition());
    telemetry.addData(
      "Front Right Encoder",
      FrontRight.getCurrentPosition()
    );
    telemetry.addData("Back Left Encoder", BackLeft.getCurrentPosition());
    telemetry.addData("Back Right Encoder", BackRight.getCurrentPosition());
    telemetry.update();


>>>>>>> Stashed changes
    while (
      !(
        !FrontLeft.isBusy() &&
        !BackLeft.isBusy() &&
        !FrontRight.isBusy() &&
        !BackRight.isBusy()
      )
    ) {}
<<<<<<< Updated upstream
    // region stop and reset encoders and reverse right motor direction
    // Reset Encoders: All Motors
=======
    
>>>>>>> Stashed changes
    FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
<<<<<<< Updated upstream
    // Right Motors: Reverse Direction
    FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
    BackRight.setDirection(DcMotorSimple.Direction.REVERSE);
    // endregion stop and reset encoders and reverse right motor direction
    // region strafe right to square B
    androidTextToSpeech.speak("Strafe Right");
    //Move Robot: Strafe Right
=======

    // Move forward 3000 ticks 79.5 in
>>>>>>> Stashed changes
    FrontLeft.setTargetPosition(tRightStrafeToSquareB);
    BackLeft.setTargetPosition(-tRightStrafeToSquareB);
    FrontRight.setTargetPosition(-tRightStrafeToSquareB);
    BackRight.setTargetPosition(tRightStrafeToSquareB);
    FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
<<<<<<< Updated upstream
    // endregion strafe right to square B
=======

    telemetry.addData("Front Left Encoder", FrontLeft.getCurrentPosition());
    telemetry.addData(
      "Front Right Encoder",
      FrontRight.getCurrentPosition()
    );
    telemetry.addData("Back Left Encoder", BackLeft.getCurrentPosition());
    telemetry.addData("Back Right Encoder", BackRight.getCurrentPosition());
    telemetry.update();


>>>>>>> Stashed changes
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

    // region strafe left

    // Move Robot: Strafe Left
    androidTextToSpeech.speak("Move Forward");
    // Set TargetPosition and ticks
<<<<<<< Updated upstream
    //Move Robot: Forward
=======
>>>>>>> Stashed changes
    FrontLeft.setTargetPosition(tForward02);
    BackLeft.setTargetPosition(tForward02);
    FrontRight.setTargetPosition(tForward02);
    BackRight.setTargetPosition(tForward02);
    // Set Mode to RUN_TO_POSITION for all motors
    FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    // endregion strafe left

    telemetry.addData("Front Left Encoder", FrontLeft.getCurrentPosition());
    telemetry.addData(
      "Front Right Encoder",
      FrontRight.getCurrentPosition()
    );
    telemetry.addData("Back Left Encoder", BackLeft.getCurrentPosition());
    telemetry.addData("Back Right Encoder", BackRight.getCurrentPosition());
    telemetry.update();


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
    // Move Robot: Strafe Left
    FrontLeft.setTargetPosition(-tStrafeLeftToPark);
    BackLeft.setTargetPosition(tStrafeLeftToPark);
    FrontRight.setTargetPosition(tStrafeLeftToPark);
    BackRight.setTargetPosition(-tStrafeLeftToPark);
    // Set Mode to RUN_TO_POSITION for all motors
    FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    // endregion strafe left

    telemetry.addData("Front Left Encoder", FrontLeft.getCurrentPosition());
    telemetry.addData(
      "Front Right Encoder",
      FrontRight.getCurrentPosition()
    );
    telemetry.addData("Back Left Encoder", BackLeft.getCurrentPosition());
    telemetry.addData("Back Right Encoder", BackRight.getCurrentPosition());
    telemetry.update();

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

    // region Drive Backward Encoder Count
    // All Motors

    androidTextToSpeech.speak("I am done");
    // endregion Drive Backward Encoder Count
    FrontLeft.setPower(0);
    BackLeft.setPower(0);
    FrontRight.setPower(0);
    BackRight.setPower (0);
  }
}
