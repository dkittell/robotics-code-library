package Java.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech;

@Autonomous(name = "Reset To Line 02 Square A(Java)", group = "")
public class ResetToLine02SquareA extends LinearOpMode {
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
    // 1 full rotation is 13 inches
    // 537.6 Encoder Counts is 1 full rotation

    double tFullRotation = 537.6;
    double iFullRotation = 13.0; // inches
    double tPerInch = tFullRotation / iFullRotation;
    double dInchesBackward = 70;
    int tBackward = (int) Math.round(tPerInch * dInchesBackward);
    double dInchesStrafeRight = 4.0; // inches
    int tStrafeRight = (int) Math.round(tPerInch * dInchesStrafeRight);
    // endregion Distance measurements to encoder Encoder Counts

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

    // Move Backwards 80 in
    FrontLeft.setTargetPosition(-tBackward);
    BackLeft.setTargetPosition(-tBackward);
    FrontRight.setTargetPosition(-tBackward);
    BackRight.setTargetPosition(-tBackward);
    FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    // endregion Encoder Initialization

    telemetry.addData("Action needed", "Please press the start triangle");
    telemetry.addData("Encoder Counts To Move", tBackward);
    telemetry.update();
    waitForStart();

    // region move left strafe
    // Run forward until all motors have achieved their Encoder Counts
    androidTextToSpeech.speak("I will move backward");
    FrontLeft.setPower(mP);
    BackLeft.setPower(mP);
    FrontRight.setPower(mP);
    BackRight.setPower(mP);
    // endregion move left strafe

    while (!(!FrontLeft.isBusy() && !BackLeft.isBusy() && !FrontRight.isBusy() && !BackRight.isBusy())) {
    }

    FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    androidTextToSpeech.speak("Strafe Right");
    FrontLeft.setTargetPosition(tStrafeRight);
    BackLeft.setTargetPosition(-tStrafeRight);
    FrontRight.setTargetPosition(-tStrafeRight);
    BackRight.setTargetPosition(tStrafeRight);
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

    androidTextToSpeech.speak("I am done");
    // Set Motor Power
    FrontLeft.setPower(0);
    BackLeft.setPower(0);
    FrontRight.setPower(0);
    BackRight.setPower(0);

    while (!(!FrontLeft.isBusy() && !BackLeft.isBusy() && !FrontRight.isBusy() && !BackRight.isBusy())) {
    }
  }
}