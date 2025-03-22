package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@Autonomous(
  name = "Basic_Iterative_Auton",
  group = "Competition",
  preselectTeleOp = "2022 Iterative"
)
public class Basic_Iterative_Auton extends OpMode {

  /* Declare OpMode members. */
  private Blinker control_Hub;
  private Blinker expansion_Hub_1;
  private Gyroscope gyro;
  private DcMotor mArm;
  private DcMotor mBackLeft;
  private DcMotor mBackRight;
  private DcMotor mDuck;
  private DcMotor mFrontLeft;
  private DcMotor mFrontRight;
  private DcMotor mPivot;
  private CRServo sIntake;
  private TouchSensor tsArm;
  private TouchSensor tsPivot;

  // region functions

  public void DriveRobotEncoder(double mp, int nDirection, double dInch) {
    int ecFR;
    double inFR;
    int ecPerInch;

    ecFR = 1120;
    inFR = 12.75;
    ecPerInch = ecFR / (int) inFR;

    int Ec = ecPerInch * (int) dInch;

    mBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    mFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    mFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
    mBackRight.setDirection(DcMotorSimple.Direction.FORWARD);

    mFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    mBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    mFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    mBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    mFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    mBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    mFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    mBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    if (nDirection == 0) {
      // Drive Forward

      // Encoder Count Drive Forward - Start
      mFrontLeft.setPower(mp);
      mBackLeft.setPower(mp);
      mFrontRight.setPower(mp);
      mBackRight.setPower(mp);
      mFrontLeft.setTargetPosition(Ec);
      mBackLeft.setTargetPosition(Ec);
      mFrontRight.setTargetPosition(Ec);
      mBackRight.setTargetPosition(Ec);
      mFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      mBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      mFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      mBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      while (
        !(
          !mFrontLeft.isBusy() &&
          !mBackLeft.isBusy() &&
          !mFrontRight.isBusy() &&
          !mBackRight.isBusy()
        )
      ) {}
      // Encoder Count Drive Forward - End
    } else if (nDirection == 1) {
      // Drive Reverse
      mFrontLeft.setPower(mp);
      mBackLeft.setPower(mp);
      mFrontRight.setPower(mp);
      mBackRight.setPower(mp);
      mFrontLeft.setTargetPosition(-Ec);
      mBackLeft.setTargetPosition(-Ec);
      mFrontRight.setTargetPosition(-Ec);
      mBackRight.setTargetPosition(-Ec);
      mFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      mBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      mFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      mBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      while (
        !(
          !mFrontLeft.isBusy() &&
          !mBackLeft.isBusy() &&
          !mFrontRight.isBusy() &&
          !mBackRight.isBusy()
        )
      ) {}
    } else if (nDirection == 2) {
      // Turn Left
      mFrontLeft.setPower(mp);
      mBackLeft.setPower(mp);
      mFrontRight.setPower(mp);
      mBackRight.setPower(mp);
      mFrontLeft.setTargetPosition(Ec);
      mBackLeft.setTargetPosition(Ec);
      mFrontRight.setTargetPosition(-Ec);
      mBackRight.setTargetPosition(-Ec);
      mFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      mBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      mFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      mBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      while (
        !(
          !mFrontLeft.isBusy() &&
          !mBackLeft.isBusy() &&
          !mFrontRight.isBusy() &&
          !mBackRight.isBusy()
        )
      ) {}
    } else if (nDirection == 3) {
      // Turn Right
      mFrontLeft.setPower(mp);
      mBackLeft.setPower(mp);
      mFrontRight.setPower(mp);
      mBackRight.setPower(mp);
      mFrontLeft.setTargetPosition(-Ec);
      mBackLeft.setTargetPosition(-Ec);
      mFrontRight.setTargetPosition(Ec);
      mBackRight.setTargetPosition(Ec);
      mFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      mBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      mFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      mBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      while (
        !(
          !mFrontLeft.isBusy() &&
          !mBackLeft.isBusy() &&
          !mFrontRight.isBusy() &&
          !mBackRight.isBusy()
        )
      ) {}
    } else if (nDirection == 4) {
      // Strafe Left
      mFrontLeft.setPower(mp);
      mBackLeft.setPower(mp);
      mFrontRight.setPower(mp);
      mBackRight.setPower(mp);
      mFrontLeft.setTargetPosition(-Ec);
      mBackLeft.setTargetPosition(Ec);
      mFrontRight.setTargetPosition(Ec);
      mBackRight.setTargetPosition(-Ec);
      mFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      mBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      mFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      mBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      while (
        !(
          !mFrontLeft.isBusy() &&
          !mBackLeft.isBusy() &&
          !mFrontRight.isBusy() &&
          !mBackRight.isBusy()
        )
      ) {}
    } else if (nDirection == 5) {
      // Strafe Right
      mFrontLeft.setPower(mp);
      mBackLeft.setPower(mp);
      mFrontRight.setPower(mp);
      mBackRight.setPower(mp);
      mFrontLeft.setTargetPosition(Ec);
      mBackLeft.setTargetPosition(-Ec);
      mFrontRight.setTargetPosition(-Ec);
      mBackRight.setTargetPosition(Ec);
      mFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      mBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      mFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      mBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      while (
        !(
          !mFrontLeft.isBusy() &&
          !mBackLeft.isBusy() &&
          !mFrontRight.isBusy() &&
          !mBackRight.isBusy()
        )
      ) {}
    } else {
      mFrontLeft.setPower(0);
      mBackLeft.setPower(0);
      mFrontRight.setPower(0);
      mBackRight.setPower(0);
    }

    telemetry.addData("Ec", Ec);
    telemetry.update();
  }

  // endregion functions

  @Override
  public void init() {
    telemetry.addData("Status", "Initialized");
    control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
    expansion_Hub_1 = hardwareMap.get(Blinker.class, "Expansion Hub 1");
    gyro = hardwareMap.get(Gyroscope.class, "gyro");
    mArm = hardwareMap.get(DcMotor.class, "mArm");
    mBackLeft = hardwareMap.get(DcMotor.class, "mBackLeft");
    mBackRight = hardwareMap.get(DcMotor.class, "mBackRight");
    mDuck = hardwareMap.get(DcMotor.class, "mDuck");
    mFrontLeft = hardwareMap.get(DcMotor.class, "mFrontLeft");
    mFrontRight = hardwareMap.get(DcMotor.class, "mFrontRight");
    mPivot = hardwareMap.get(DcMotor.class, "mPivot");
    sIntake = hardwareMap.get(CRServo.class, "sIntake");
    tsArm = hardwareMap.get(TouchSensor.class, "tsArm");
    tsPivot = hardwareMap.get(TouchSensor.class, "tsPivot");
  }

  /*
   * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
   */
  @Override
  public void init_loop() {}

  /*
   * Code to run ONCE when the driver hits PLAY
   */
  @Override
  public void start() {
    telemetry.speak("Forward", null, null);
    DriveRobotEncoder(0.5, 0, 13);

    telemetry.speak("Backward", null, null);
    DriveRobotEncoder(0.5, 1, 13);

    telemetry.speak("Strafe Left", null, null);
    DriveRobotEncoder(0.5, 4, 13);

    telemetry.speak("Strafe Right", null, null);
    DriveRobotEncoder(0.5, 5, 13);

mFrontLeft.setPower(0);

    mBackLeft.setPower(0);
    mFrontRight.setPower(0);
    mBackRight.setPower(0);
    
  }

  /*
   * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
   */
  @Override
  public void loop() {}

  /*
   * Code to run ONCE after the driver hits STOP
   */
  @Override
  public void stop() {}
}
