package Java.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech;

@Autonomous(name = "Mark Auton 01 Square A (Java)", group = "")
public class Mark_Auton01_SquareA extends LinearOpMode {
  private AndroidTextToSpeech androidTextToSpeech;
  private DcMotor FrontRight;
  private DcMotor BackRight;
  private DcMotor FrontLeft;
  private DcMotor BackLeft;

  /**
   * This function is executed when this Op Mode is selected from the Driver
   * Station.
   */
  @Override
  public void runOpMode() {
    // region Set Variables

    // region Distance measurements to encoder Encoder Counts

    // 1 full rotation is ? inches
    // 1120 Encoder counts is 1 full rotation
    // https://www.andymark.com/products/neverest-classic-40-gearmotor
    double ecFullRotation = 1120; // Elon M Robot
    double iFullRotation = 13.0; // Inches - Elon M Robot

    // 1 full rotation is 13 inches
    // 537.6 Encoder counts is 1 full rotation
    // https://www.andymark.com/products/neverest-orbital-20-gearmotor
    // double ecFullRotation = 537.6; // Mark Robot
    // double iFullRotation = 13.0; // Inches - Mark Robot
    double ecPerInch = ecFullRotation / iFullRotation;
    double dInchesStrafeRightToSquareA = 77;
    int ecRightStrafeToSquareA = (int) Math.round(ecPerInch * dInchesStrafeRightToSquareA);
    double dInchesForward01 = 19.0; // inches
    int ecForward01 = (int) Math.round(ecPerInch * dInchesForward01);
    double dInchesForward02 = 35.0; // inches
    int ecForward02 = (int) Math.round(ecPerInch * dInchesForward02);
    double dInchesBackward01 = 25.0; // inches
    int ecBackward01 = (int) Math.round(ecPerInch * dInchesBackward01);
    double dInchesBackward02 = 13.0; // inches
    int ecBackward02 = (int) Math.round(ecPerInch * dInchesBackward02);
    // endregion Distance measurements to encoder Encoder Counts

    // Motor Power Variable
    double mP = 0.5;

    // Driving Encoder Encoder Counts
    // tRS - Encoder Counts Right Strafe
    // int tRS = tRightStrafeToSquareB;
    // tStrafeLeftToPark - Encoder Counts Left Strafe To Park
    // int tStrafeLeftToPark = 5376;
    // tF - Encoder Counts Forward
    // int tF = tForward
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

    // Move forward 3000 Encoder Counts 79.5 in
    FrontLeft.setTargetPosition(ecForward01);
    BackLeft.setTargetPosition(ecForward01);
    FrontRight.setTargetPosition(ecForward01);
    BackRight.setTargetPosition(ecForward01);
    FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    // endregion Encoder Initialization

    telemetry.addData("Action needed", "Please press the start triangle");
    telemetry.addData("Encoder counts To Move", ecForward01);
    telemetry.update();
    waitForStart();

    // region move forward1
    // Run forward until all motors have achieved their Encoder Counts
    androidTextToSpeech.speak("I will move forward");
    FrontLeft.setPower(mP);
    BackLeft.setPower(mP);
    FrontRight.setPower(mP);
    BackRight.setPower(mP);
    // endregion move forward1

    while (!(!FrontLeft.isBusy() && !BackLeft.isBusy() && !FrontRight.isBusy() && !BackRight.isBusy())) {
    }

    FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    // Move backward
    androidTextToSpeech.speak("I will move backward");
    FrontLeft.setTargetPosition(-ecBackward01);
    BackLeft.setTargetPosition(-ecBackward01);
    FrontRight.setTargetPosition(-ecBackward01);
    BackRight.setTargetPosition(-ecBackward01);
    FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    while (!(!FrontLeft.isBusy() && !BackLeft.isBusy() && !FrontRight.isBusy() && !BackRight.isBusy())) {
    }
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
    androidTextToSpeech.speak("I will Strafe Right");
    // Set TargetPosition and Encoder Counts
    FrontLeft.setTargetPosition(ecRightStrafeToSquareA);
    BackLeft.setTargetPosition(-ecRightStrafeToSquareA);
    FrontRight.setTargetPosition(-ecRightStrafeToSquareA);
    BackRight.setTargetPosition(ecRightStrafeToSquareA);
    // Set Mode to RUN_TO_POSITION for all motors
    FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    // endregion strafe left

    while (!(!FrontLeft.isBusy() && !BackLeft.isBusy() && !FrontRight.isBusy() && !BackRight.isBusy())) {
    }

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

    androidTextToSpeech.speak("I will move backwards");
    // Set TargetPosition and Encoder Counts
    FrontLeft.setTargetPosition(-ecBackward02);
    BackLeft.setTargetPosition(-ecBackward02);
    FrontRight.setTargetPosition(-ecBackward02);
    BackRight.setTargetPosition(-ecBackward02);
    // Set Mode to RUN_TO_POSITION for all motors
    FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    // endregion strafe left

    while (!(!FrontLeft.isBusy() && !BackLeft.isBusy() && !FrontRight.isBusy() && !BackRight.isBusy())) {
    }

    sleep(2000);

    // region reset encode and reverse direction
    // Reset encoders reverse motors right set
    // Encoder Counts tell motors to move
    //
    FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
    BackRight.setDirection(DcMotorSimple.Direction.REVERSE);

    // endregion reset encode and reverse direction

    androidTextToSpeech.speak("I will move forward");
    // Set TargetPosition and Encoder Counts
    FrontLeft.setTargetPosition(ecForward02);
    BackLeft.setTargetPosition(ecForward02);
    FrontRight.setTargetPosition(ecForward02);
    BackRight.setTargetPosition(ecForward02);
    // Set Mode to RUN_TO_POSITION for all motors
    FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    // endregion strafe left

    while (!(!FrontLeft.isBusy() && !BackLeft.isBusy() && !FrontRight.isBusy() && !BackRight.isBusy())) {
    }

    // region Drive Backward Encoder Count
    // All Motors

    androidTextToSpeech.speak("I am done");
    // endregion Drive Backward Encoder Count
    FrontLeft.setPower(0);
    BackLeft.setPower(0);
    FrontRight.setPower(0);
    BackRight.setPower(0);
  }
}
