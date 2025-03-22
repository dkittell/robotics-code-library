package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "2022 Iterative OpMode", group = "Iterative Opmode")
public class BasicOpMode_Iterative extends OpMode {

  // Declare OpMode members.
  private ElapsedTime runtime = new ElapsedTime();

  private DcMotor m_Arm = null;
  private CRServo Grab_One = null;
  private CRServo Grab_Two = null;
  private CRServo Grab_Three = null;

  private DcMotor Back_Left = null;
  private DcMotor Front_Left = null;
  private DcMotor Front_Right = null;
  private DcMotor Back_Right = null;

  public double Gear;
  public int Dpaduplast;
  public int Dpaddownlast;
  public double Gear2;
  public int Dpaddownlast2;
  public int Dpaduplast2;
  public int eU;
  public double inchL;
  public double mP;
  public double inchM;
  public double inchH;
  public double inchG;

  /*
   * Code to run ONCE when the driver hits INIT
   */
  @Override
  public void init() {
    Gear = 0.1;
    Dpaduplast = 0;
    Dpaddownlast = 0;

    // Initialize the hardware variables. Note that the strings used here as parameters
    // to 'get' must correspond to the names assigned during the robot configuration
    // step (using the FTC Robot Controller app on the phone).

    Back_Left = hardwareMap.get(DcMotor.class, "Back_Left");
    Front_Left = hardwareMap.get(DcMotor.class, "Front_Left");
    Front_Right = hardwareMap.get(DcMotor.class, "Front_Right");
    Back_Right = hardwareMap.get(DcMotor.class, "Back_Right");

    m_Arm = hardwareMap.get(DcMotor.class, "m_Arm");
    Grab_One = hardwareMap.get(CRServo.class, "Grab_One");
    Grab_Two = hardwareMap.get(CRServo.class, "Grab_Two");
    Grab_Three = hardwareMap.get(CRServo.class, "Grab_Three");

    // Most robots need the motor on one side to be reversed to drive forward
    // Reverse the motor that runs backwards when connected directly to the battery

    m_Arm.setDirection(DcMotorSimple.Direction.REVERSE);
    Back_Left.setDirection(DcMotorSimple.Direction.REVERSE);
    Front_Left.setDirection(DcMotorSimple.Direction.REVERSE);
    Front_Right.setDirection(DcMotorSimple.Direction.FORWARD);
    Back_Right.setDirection(DcMotorSimple.Direction.FORWARD);

    Front_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Front_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Back_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Back_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    Front_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    Front_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    Back_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    Back_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    Front_Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    Back_Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    Front_Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    Back_Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    // region Set values to variables
    Gear = 0.1;
    Dpaddownlast = 0;
    Dpaduplast = 0;
    Gear2 = 0.1;
    Dpaddownlast2 = 0;
    Dpaduplast2 = 0;
    eU = 637;
    // 637 x 1.9 = 13.75 in
    inchL = eU * 1.9;
    mP = 1.0;
    // 637 x 3.3 = 23.75 in
    inchM = eU * 3.3;
    // 637 x 4.8 = 34.03125 in
    inchH = eU * 4.8;
    // 637 x 0.3 =
    inchG = eU * 0.3;
    // endregion Set values to variables

    // Tell the driver that initialization is complete.
    telemetry.addData("Status", "Initialized");
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
    runtime.reset();
  }

  /*
   * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
   */
  @Override
  public void loop() {
    // Setup a variable for each drive wheel to save power level for telemetry
    // double leftPower;
    // double rightPower;
    // region Drive Code Begin (up and down dpad)
    // if dpad pressed up, increase gear 0.79, limit to 0.79 power(drive)
    if (Dpaduplast == 0) {
      if (Gear < 0.79) {
        if (gamepad1.dpad_up) {
          Dpaduplast = 1;
          Gear += 0.79;
        }
      }
    } else if (!gamepad1.dpad_up) {
      Dpaduplast = 0;
    }
    // if dpad pressed down, decrease gear 0.6, limit to 0.3 power(drive)
    if (Dpaddownlast == 0) {
      if (Gear > 0.3) {
        if (gamepad1.dpad_down) {
          Gear += -0.6;
          Dpaddownlast = 1;
        }
      }
    } else if (!gamepad1.dpad_down) {
      Dpaddownlast = 0;
    }

    // region Start of Arm Joystick
    // if dpad pressed up, increase power by 0.1, limit power to 0.79 (arm)
    if (Dpaduplast2 == 0) {
      if (Gear2 < 0.79) {
        if (gamepad2.dpad_up) {
          Dpaduplast2 = 1;
          Gear2 += 0.1;
        }
      }
    } else if (!gamepad2.dpad_up) {
      Dpaduplast2 = 0;
    }
    // if dpad pressed down, decrease power by 0.1, limit power to 0.11 (arm)
    if (Dpaddownlast2 == 0) {
      if (Gear2 > 0.11) {
        if (gamepad2.dpad_down) {
          Gear2 += -0.1;
          Dpaddownlast2 = 1;
        }
      }
    } else if (!gamepad2.dpad_down) {
      Dpaddownlast2 = 0;
    }
    // endregion Start of Arm Joystick
    // Choose to drive using either Tank Mode, or POV Mode
    // Comment out the method that's not used. The default below is POV.

    // POV Mode uses left stick to go forward, and right stick to turn.
    // - This uses basic math to combine motions and is easier to drive straight.
    // double drive = -gamepad1.left_stick_y;
    // double turn = gamepad1.right_stick_x;
    // leftPower = Range.clip(drive + turn, -1.0, 1.0);
    // rightPower = Range.clip(drive - turn, -1.0, 1.0);

    // telemetry.addData("Gear_Arm", Gear2);
    // if the right trigger is pressed move servos forward
    if (gamepad2.right_trigger > 0.5) {
      Grab_One.setPower(gamepad2.right_trigger);
      Grab_Two.setPower(gamepad2.right_trigger);
      // if the left trigger is pressed move servos backwards
    } else if (gamepad2.left_trigger > 0.5) {
      Grab_One.setPower(-gamepad2.left_trigger);
      Grab_Two.setPower(-gamepad2.left_trigger);
      // if nothing pressed do nothing
    } else {
      Grab_One.setPower(0);
      Grab_Two.setPower(0);
    }
    // Tank Mode uses one stick to control each wheel.
    // - This requires no math, but it is hard to drive forward slowly and keep
    // straight.
    // leftPower = -gamepad1.left_stick_y ;
    // rightPower = -gamepad1.right_stick_y ;

    // Send calculated power to wheels
    // Front_Left.setPower(leftPower);
    // Front_Right.setPower(rightPower);
    // Back_Left.setPower(leftPower);
    // Back_Right.setPower(rightPower);

    telemetry.addData("Time Passed", getRuntime());
    telemetry.addData("Gear", (Gear * 10));
    // moves front right wheel if sticks pressed
    Front_Right.setPower(
      (-gamepad1.left_stick_y * Gear - gamepad1.right_stick_x * Gear) -
      gamepad1.left_stick_x *
      Gear
    );
    // moves front left wheel if sticks are pressed
    Front_Left.setPower(
      -gamepad1.left_stick_y *
      Gear +
      gamepad1.right_stick_x *
      Gear +
      gamepad1.left_stick_x *
      Gear
    );
    // moves back left wheel if sticks are pressed
    Back_Left.setPower(
      (-gamepad1.left_stick_y * Gear + gamepad1.right_stick_x * Gear) -
      gamepad1.left_stick_x *
      Gear
    );
    // moves back right wheel if sticks are pressed
    Back_Right.setPower(
      (-gamepad1.left_stick_y * Gear - gamepad1.right_stick_x * Gear) +
      gamepad1.left_stick_x *
      Gear
    );
    telemetry.addData("Gear", Gear);

    // Move Arm to low junction
    if (gamepad2.a) {
      m_Arm.setPower(mP);
      m_Arm.setTargetPosition((int) inchL);
      m_Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      // sleep(1000);
    }
    // Move Arm to medium junction
    if (gamepad2.b) {
      m_Arm.setPower(mP);
      m_Arm.setTargetPosition((int) inchM);
      m_Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      // sleep(1000);
    }
    // Move Arm to high junction
    if (gamepad2.x) {
      m_Arm.setPower(mP);
      m_Arm.setTargetPosition((int) inchH);
      m_Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      // sleep(5000);
    }
    // Move Arm to ground junction
    if (gamepad2.y) {
      m_Arm.setPower(mP);
      m_Arm.setTargetPosition((int) inchG);
      m_Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      // sleep(5000);
    }
    // Send arm to the bottom
    if (gamepad2.right_bumper) {
      m_Arm.setPower(0);
      m_Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    telemetry.addData("Encoder's Elapsed", m_Arm.getCurrentPosition());
  }

  /*
   * Code to run ONCE after the driver hits STOP
   */
  @Override
  public void stop() {}
}
