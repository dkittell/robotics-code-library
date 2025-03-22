package org.firstinspires.ftc.teamcode.Competition.Teleop;

//region Imports
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//endregion Imports

// @Disabled
@TeleOp(name = "Single Operator", group = "Iterative Opmode")
public class SingleOperator extends OpMode {

  // Declare OpMode members.
  private ElapsedTime runtime = new ElapsedTime();

  private CRServo sGrabOne = null;
  private CRServo sGrabTwo = null;
  private CRServo sGrabThree = null;

  private DcMotor mArm = null;
  private DcMotor mBackLeft = null;
  private DcMotor mFrontLeft = null;
  private DcMotor mFrontRight = null;
  private DcMotor mBackRight = null;

  public double dDriverGear;
  public int nDriverDpadUpLast;
  public int nDriverDpadDownLast;
  public double dOperatorGear;
  public int nOperatorDpadDownLast;
  public int nOperatorDpadUpLast;
  public int eU;
  public double inchL;
  public double mP;
  public double inchM;
  public double inchH;
  public double inchG;
  public boolean toggle;

  public int ecArmCurrent;
  private RevBlinkinLedDriver blinkin;

  /*
   * Code to run ONCE when the driver hits INIT
   */
  @Override
  public void init() {
    // Initialize the hardware variables. Note that the strings used here as parameters
    // to 'get' must correspond to the names assigned during the robot configuration
    // step (using the FTC Robot Controller app on the phone).

    mBackLeft = hardwareMap.get(DcMotor.class, "mBackLeft");
    mFrontLeft = hardwareMap.get(DcMotor.class, "mFrontLeft");
    mFrontRight = hardwareMap.get(DcMotor.class, "mFrontRight");
    mBackRight = hardwareMap.get(DcMotor.class, "mBackRight");
    mArm = hardwareMap.get(DcMotor.class, "mArm");

    sGrabOne = hardwareMap.get(CRServo.class, "sGrabOne");
    sGrabTwo = hardwareMap.get(CRServo.class, "sGrabTwo");
    sGrabThree = hardwareMap.get(CRServo.class, "sGrabThree");

    blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

    // Most robots need the motor on one side to be reversed to drive forward
    // Reverse the motor that runs backwards when connected directly to the battery

    mArm.setDirection(DcMotorSimple.Direction.REVERSE);
    mBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    mFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    mFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
    mBackRight.setDirection(DcMotorSimple.Direction.FORWARD);

    mFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    mFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    mBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    mBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    mArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    mFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    mFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    mBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    mBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    mArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    mFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    mBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    mFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    mBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    mArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    // region Set values to variables
    dDriverGear = 0.8;
    nDriverDpadUpLast = 0; // Last known value for dpadUp - Driver Controller
    nDriverDpadDownLast = 0; // Last known value for dpadDown - Driver Controller

    // region Arm Code Variables/Constants
    dOperatorGear = 0.1;
    nOperatorDpadDownLast = 0; // Last known value for dpadUp - Operator Controller
    nOperatorDpadUpLast = 0; // Last known value for dpadDown - Operator Controller
    eU = 637; // Encoder count for full rotation - Arm

    // 637 x 1.9 = 13.75 in
    inchL = eU * 2.2;
    mP = 1.0;
    // 637 x 3.3 = 23.75 in
    inchM = eU * 3.5;
    // 637 x 4.8 = 34.03125 in
    inchH = eU * 4.8;
    // 637 x 0.3 =
    inchG = eU * 0.7;
    // endregion Arm Code Variables/Constants

    // endregion Set values to variables

    // Tell the driver that initialization is complete.
    telemetry.addData("Status", "Initialized");
  }

  /*
   * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
   */
  @Override
  public void init_loop() {
    // Insert code here to initialize for better telop work
  }

  /*
   * Code to run ONCE when the driver hits PLAY
   */
  @Override
  public void start() {
    runtime.reset();
    boolean toggle = false;
  }

  /*
   * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
   */
  @Override
  public void loop() {
    ecArmCurrent = mArm.getCurrentPosition();

    // Setup a variable for each drive wheel to save power level for telemetry
    
    // if (gamepad1.back && !toggle) {
      
    //   mFrontLeft.setPower(
    //   -gamepad1.left_stick_y *
    //   dDriverGear +
    //   gamepad1.right_stick_x *
    //   dDriverGear +
    //   gamepad1.left_stick_x *
    //   dDriverGear
    // );
    // mFrontRight.setPower(
    //   (
    //     -gamepad1.left_stick_y *
    //     dDriverGear -
    //     gamepad1.right_stick_x *
    //     dDriverGear
    //   ) -
    //   gamepad1.left_stick_x *
    //   dDriverGear
    // );

    // mBackLeft.setPower(
    //   (
    //     -gamepad1.left_stick_y *
    //     dDriverGear +
    //     gamepad1.right_stick_x *
    //     dDriverGear
    //   ) -
    //   gamepad1.left_stick_x *
    //   dDriverGear
    // );

    // mBackRight.setPower(
    //   (
    //     -gamepad1.left_stick_y *
    //     dDriverGear -
    //     gamepad1.right_stick_x *
    //     dDriverGear
    //   ) +
    //   gamepad1.left_stick_x *
    //   dDriverGear
    // );
      
    //   if (gamepad1.right_trigger > 0.5) {
    //   sGrabOne.setPower(gamepad1.right_trigger);
    //   sGrabTwo.setPower(-gamepad1.right_trigger);
    //   // if the left trigger is pressed move servos backwards
    // } else if (gamepad1.left_trigger > 0.5) {
    //   sGrabOne.setPower(-gamepad1.left_trigger);
    //   sGrabTwo.setPower(gamepad1.left_trigger);
    //   // if nothing pressed do nothing
    // } else {
    //   sGrabOne.setPower(0);
    //   sGrabTwo.setPower(0);
    // }
    
    // if (gamepad1.dpad_up) {
    //   mArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    //   mArm.setPower(-0.5);
    // }
    
    // if (gamepad1.dpad_down) {
    // mArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    //   mArm.setPower(0.5);
    // }

    // // Move Arm to low junction
    // if (gamepad1.a) {
    //   mArm.setPower(mP);
    //   mArm.setTargetPosition((int) inchL);
    //   mArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    // }

    // // Move Arm to medium junction
    // if (gamepad1.b) {
    //   mArm.setPower(mP);
    //   mArm.setTargetPosition((int) inchM);
    //   mArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    // }
    // // Move Arm to high junction
    // if (gamepad1.x) {
    //   mArm.setPower(mP);
    //   mArm.setTargetPosition((int) inchH);
    //   mArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    // }
    // // Move Arm to ground junction
    // if (gamepad1.y) {
    //   mArm.setPower(mP);
    //   mArm.setTargetPosition((int) inchG);
    //   mArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    // }
    // // Send arm to the bottom
    // if (gamepad1.right_bumper) {
    //   mArm.setPower(mP);
    //   mArm.setTargetPosition(1000);
    //   mArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    // }
    // toggle = true;
    
    // } else if (!gamepad1.back)  toggle = false; 
      
    

    // region Drive Code (up and down dpad)
    if (nDriverDpadUpLast == 0) {
      if (dDriverGear < 0.89) {
        if (gamepad2.dpad_up) {
          nDriverDpadUpLast = 1;
          dDriverGear += 0.1;
        }
      }
    } else if (!gamepad2.dpad_up) {
      nDriverDpadUpLast = 0;
    }
    if (nDriverDpadDownLast == 0) {
      if (dDriverGear > 0.11) {
        if (gamepad2.dpad_down) {
          dDriverGear += -0.1;
          nDriverDpadDownLast = 1;
        }
      }
    } else if (!gamepad2.dpad_down) {
      nDriverDpadDownLast = 0;
    }
    // endregion Drive Code (up and down dpad)

    // region Operator Code Begin (up and down dpad)
    // if (dOperatorGear < 0.79) {
    //   if (gamepad2.dpad_up) {
    //     dOperatorGear += 0.1;
    //   }
    // }

    // if (dOperatorGear > 0.11) {
    //   if (gamepad2.dpad_down) {
    //     dOperatorGear += -0.1;
    //   }
    // }
    // endregion Operator Code Begin (up and down dpad)

    // region Operator Controller
    if (gamepad2.right_trigger > 0.5) {
      sGrabOne.setPower(gamepad2.right_trigger);
      sGrabTwo.setPower(-gamepad2.right_trigger);
      // if the left trigger is pressed move servos backwards
    } else if (gamepad2.left_trigger > 0.5) {
      sGrabOne.setPower(-gamepad2.left_trigger);
      sGrabTwo.setPower(gamepad2.left_trigger);
      // if nothing pressed do nothing
    } else {
      sGrabOne.setPower(0);
      sGrabTwo.setPower(0);
    }
    //endregion Operator Controller

    // region Driver Controller
    

    // region Simply Drive Robot
    mFrontLeft.setPower(
      -gamepad2.left_stick_y *
      dDriverGear +
      gamepad2.right_stick_x *
      dDriverGear +
      gamepad2.left_stick_x *
      dDriverGear
    );
    mFrontRight.setPower(
      (
        -gamepad2.left_stick_y *
        dDriverGear -
        gamepad2.right_stick_x *
        dDriverGear
      ) -
      gamepad2.left_stick_x *
      dDriverGear
    );

    mBackLeft.setPower(
      (
        -gamepad2.left_stick_y *
        dDriverGear +
        gamepad2.right_stick_x *
        dDriverGear
      ) -
      gamepad2.left_stick_x *
      dDriverGear
    );

    mBackRight.setPower(
      (
        -gamepad2.left_stick_y *
        dDriverGear -
        gamepad2.right_stick_x *
        dDriverGear
      ) +
      gamepad2.left_stick_x *
      dDriverGear
    );
    // endregion Simply Drive Robot
    // endregion Driver Controller

    //region Operator Controller

    // If you hold left bumper you can move the arm with the left joystick
   if (gamepad2.left_bumper) {
       mArm.setPower(mP);
      mArm.setTargetPosition(200);
      mArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      sGrabOne.setPower(0.5);
      sGrabTwo.setPower(-0.5);
    }
    

    // Move Arm to low junction
    if (gamepad2.a) {
      mArm.setPower(mP);
      mArm.setTargetPosition((int) inchL);
      mArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // Move Arm to medium junction
    if (gamepad2.b) {
      mArm.setPower(mP);
      mArm.setTargetPosition((int) inchM);
      mArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    // Move Arm to high junction
    if (gamepad2.x) {
      mArm.setPower(mP);
      mArm.setTargetPosition((int) inchH);
      mArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    // Move Arm to ground junction
    if (gamepad2.y) {
      mArm.setPower(mP);
      mArm.setTargetPosition((int) inchG);
      mArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    // Send arm to the bottom
   
    //endregion Operator Controller

    telemetry.addData("Time Passed", getRuntime());
    telemetry.addData("Gear_Drive", dDriverGear);
    telemetry.addData("Gear_Arm", dOperatorGear);
    telemetry.addData("Encoder's Elapsed", mArm.getCurrentPosition());
  }

  /*
   * Code to run ONCE after the driver hits STOP
   */
  @Override
  public void stop() {}
}
