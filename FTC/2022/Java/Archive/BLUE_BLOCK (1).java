package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Autonomous(name = "BLUE_BLOCK", group = "basic")

public class BLUE_BLOCK extends LinearOpMode{

private ElapsedTime runtime = new ElapsedTime();
    //Establishing DC Motors
    private DcMotor FrontLeft;
    private DcMotor FrontRight;
    private DcMotor BackLeft;
    private DcMotor BackRight;
    private DcMotor Middle;
    private DcMotor RightIntake;
    private DcMotor LeftIntake;
    private DcMotor ArmMotor;
    private Servo ArmServo;
    private Servo Gripper;
    private TouchSensor ArmSensor;

    static final double     FORWARD_RIGHT_SPEED = 0.6;
    static final double     FORWARD_LEFT_SPEED = -0.6;
    static final double     TURN_SPEED    = 0.5;
    static final double     STRAFE_SPEED    = -0.8;
    static final double     ARM_SPEED = 1;
    static final double     INTAKE = -0.8;
    static final double     ZERO    = 0;
    static final double     IN = 1;
    static final double     OUT = 0;
    static final double     CLOSED = 0.42;
    static final double     OPEN = 0.58;
    
    @Override
    public void runOpMode(){

        //Assigning Private DC Motors To Hardware Maps
        FrontLeft = hardwareMap.dcMotor.get("Front Left");
        FrontRight = hardwareMap.dcMotor.get("Front Right");
        BackLeft = hardwareMap.dcMotor.get("Back Left");
        BackRight = hardwareMap.dcMotor.get("Back Right");
        Middle = hardwareMap.dcMotor.get("Middle");
        RightIntake = hardwareMap.dcMotor.get("Intake Right");
        LeftIntake = hardwareMap.dcMotor.get("Intake Left");
        ArmMotor = hardwareMap.dcMotor.get("Arm Motor");
        ArmServo = hardwareMap.servo.get("Arm Servo");
        Gripper = hardwareMap.servo.get("Gripper");
        ArmSensor = hardwareMap.touchSensor.get("Arm Sensor");

        //Assigning Initial Motor Power To 0
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
        Middle.setPower(0);
        RightIntake.setPower(0);
        LeftIntake.setPower(0);
        ArmMotor.setPower(0);
        ArmServo.setPosition(IN);
        Gripper.setPosition(OPEN);

        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1:  Drive forward for 3 seconds
        FrontLeft.setPower(ZERO);
        FrontRight.setPower(ZERO);
        BackLeft.setPower(ZERO);
        BackRight.setPower(ZERO);
        Middle.setPower(STRAFE_SPEED);
        LeftIntake.setPower(ZERO);
        RightIntake.setPower(ZERO);
        ArmMotor.setPower(ZERO);
        ArmServo.setPosition(IN);
        Gripper.setPosition(CLOSED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.1)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
       }
       
        FrontLeft.setPower(ZERO);
        FrontRight.setPower(ZERO);
        BackLeft.setPower(ZERO);
        BackRight.setPower(ZERO);
        Middle.setPower(ZERO);
        LeftIntake.setPower(ZERO);
        RightIntake.setPower(ZERO);
        ArmMotor.setPower(ZERO);
        ArmServo.setPosition(IN);
        Gripper.setPosition(CLOSED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
       }
        FrontLeft.setPower(FORWARD_LEFT_SPEED);
        FrontRight.setPower(FORWARD_RIGHT_SPEED);
        BackLeft.setPower(FORWARD_LEFT_SPEED);
        BackRight.setPower(FORWARD_RIGHT_SPEED);
        Middle.setPower(ZERO);
        LeftIntake.setPower(INTAKE);
        RightIntake.setPower(INTAKE);
        ArmMotor.setPower(ZERO);
        ArmServo.setPosition(IN);
        Gripper.setPosition(CLOSED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
       }
        
        FrontLeft.setPower(ZERO);
        FrontRight.setPower(ZERO);
        BackLeft.setPower(ZERO);
        BackRight.setPower(ZERO);
        Middle.setPower(ZERO);
        LeftIntake.setPower(INTAKE);
        RightIntake.setPower(INTAKE);
        ArmMotor.setPower(ZERO);
        ArmServo.setPosition(IN);
        Gripper.setPosition(CLOSED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
       }
        
        FrontLeft.setPower(ZERO);
        FrontRight.setPower(ZERO);
        BackLeft.setPower(ZERO);
        BackRight.setPower(ZERO);
        Middle.setPower(-STRAFE_SPEED);
        LeftIntake.setPower(INTAKE);
        RightIntake.setPower(INTAKE);
        ArmMotor.setPower(ZERO);
        ArmServo.setPosition(IN);
        Gripper.setPosition(CLOSED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.3)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
       }
        
        FrontLeft.setPower(ZERO);
        FrontRight.setPower(ZERO);
        BackLeft.setPower(ZERO);
        BackRight.setPower(ZERO);
        Middle.setPower(ZERO);
        LeftIntake.setPower(ZERO);
        RightIntake.setPower(ZERO);
        ArmMotor.setPower(ZERO);
        ArmServo.setPosition(IN);
        Gripper.setPosition(CLOSED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
       }       
       
        FrontLeft.setPower(FORWARD_LEFT_SPEED);
        FrontRight.setPower(FORWARD_RIGHT_SPEED);
        BackLeft.setPower(FORWARD_LEFT_SPEED);
        BackRight.setPower(FORWARD_RIGHT_SPEED);
        Middle.setPower(ZERO);
        LeftIntake.setPower(ZERO);
        RightIntake.setPower(ZERO);
        ArmMotor.setPower(ZERO);
        ArmServo.setPosition(IN);
        Gripper.setPosition(CLOSED);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.8)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
       } 
        
        
    }
}
