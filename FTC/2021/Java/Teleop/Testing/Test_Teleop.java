package org.firstinspires.ftc.Testing;
// Importing Neccesary Libraries 
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Disabled
@TeleOp(name = "Test_Teleop")

public class Test_Teleop extends LinearOpMode {
 //Defining Motors and Servos
  private DcMotor m_Arm;
  private CRServo Grab_One;
    private CRServo Grab_Two;
    private CRServo Grab_Three;
    private DcMotor Back_Left;
    private DcMotor Front_Left;
    private DcMotor Front_Right;
    private DcMotor Back_Right;
    
    @Override
    public void runOpMode() {
    // declaring variables
    double Gear;
    int Dpaddownlast;
    int Dpaduplast;
    double Gear2;
    int Dpaddownlast2;
    int Dpaduplast2;
    // hardware maps
  Back_Left = hardwareMap.get(DcMotor.class, "Back_Left");
  Front_Left = hardwareMap.get(DcMotor.class, "Front_Left");
  Front_Right = hardwareMap.get(DcMotor.class, "Front_Right");
  Back_Right = hardwareMap.get(DcMotor.class, "Back_Right");
  m_Arm = hardwareMap.get(DcMotor.class, "m_Arm");
  Grab_One = hardwareMap.get(CRServo.class, "Grab_One");
  Grab_Two = hardwareMap.get(CRServo.class, "Grab_Two");
  Grab_Three = hardwareMap.get(CRServo.class, "Grab_Three");
  // Put Init lines
  // Reversing the Motor Directions
  Back_Left.setDirection(DcMotorSimple.Direction.REVERSE);
  Front_Left.setDirection(DcMotorSimple.Direction.REVERSE);
  // Stop and Reseting Encoders
  Front_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  Front_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  Back_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  Back_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  m_Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  // Run Using Encoders
  Front_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
  Front_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
  Back_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
  Back_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
  // Brakes for the Motors
  m_Arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  Back_Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  Back_Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  Front_Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  Front_Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  // Assign Values to Variables
  Gear = 0.1;
  Dpaddownlast = 0;
  Dpaduplast = 0;
  Gear2 = 0.1;
  Dpaddownlast2 = 0;
  Dpaduplast2 = 0;
  
  waitForStart();
  if (opModeIsActive()) {
    while (opModeIsActive()); {
    // Put Teleop Code Under Here
    // If dpad pressed up, gear goes up 0.1 = faster speed
    if (Dpaduplast == 0) {
      if (Gear < 0.79) {
        if (gamepad1.dpad_up) {
          Dpaduplast = 1;
          Gear += 0.1;
        }
      }
    } else if (!gamepad1.dpad_up) {
      Dpaduplast = 0;
      
    }
    // If dpad pressed down, gear goes down 0.1 = slower speed
    if (Dpaddownlast == 0) {
      if (Gear > 0.11) {
        if (gamepad1.dpad_down) {
          Gear += -0.1;
          Dpaddownlast = 1;
        }
      }
    } else if (!gamepad1.dpad_down) {
      Dpaddownlast = 0;
        Front_Right.setPower((-gamepad1.left_stick_y * Gear - gamepad1.right_stick_x * Gear) - gamepad1.left_stick_x * Gear);
        Front_Left.setPower(-gamepad1.left_stick_y * Gear + gamepad1.right_stick_x * Gear + gamepad1.left_stick_x * Gear);
        Back_Left.setPower((-gamepad1.left_stick_y * Gear + gamepad1.right_stick_x * Gear) - gamepad1.left_stick_x * Gear);
        Back_Right.setPower((-gamepad1.left_stick_y * Gear - gamepad1.right_stick_x * Gear) + gamepad1.left_stick_x * Gear);
        telemetry.addData("Gear_Drive" , Gear);
        // end of the drive code
        // arm code if dpad pressed up, gear increase by 0.1, arm go faster
        if (Dpaduplast2 == 0) {
          if (Gear < 0.79) {
            if (gamepad2.dpad_up) {
              Dpaduplast2 = 1;
              Gear2 += 0.1;
            }
          }
        } else if (!gamepad2.dpad_up) {
          Dpaduplast2 = 0;
        }
        // arm code if dpad pressed down, gear decrease by 0.1, arm go slower
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
      m_Arm.setPower((gamepad2.left_stick_y * Gear));
      telemetry.addData("Gear_Arm", Gear2);
      // Servo code if press a, all servos move forward
      // if press B, all servos move backwards
      // if press nothing servos dont move
      if (gamepad2.right_bumper) {
          Grab_One.setPower(1);
          Grab_Two.setPower(1);
          Grab_Three.setPower(1);
        } else if (gamepad2.left_bumper) {
          Grab_One.setPower(-1);
          Grab_Two.setPower(-1);
          Grab_Three.setPower(-1);
        } else {
          Grab_One.setPower(0);
          Grab_Two.setPower(0);
          Grab_Three.setPower(0);
        }
        telemetry.addData("Time Passed", getRuntime());
        telemetry.update();
    }
    }
     }
  }
  }
  
 
