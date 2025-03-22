package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Comp Bot Teleop H-Drive", group = "")
public class HDRIVEARCADE extends OpMode {

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
    private Servo Platform;
    private TouchSensor ArmSensor;

    @Override
    public void init(){
        
        FrontLeft = hardwareMap.dcMotor.get("Front Left");
        FrontRight = hardwareMap.dcMotor.get("Front Right");
        BackLeft = hardwareMap.dcMotor.get("Back Left");
        BackRight = hardwareMap.dcMotor.get("Back Right");
        Middle = hardwareMap.dcMotor.get("Middle");
        RightIntake = hardwareMap.dcMotor.get("Intake Right");
        LeftIntake = hardwareMap.dcMotor.get("Intake Left");
        ArmMotor = hardwareMap.dcMotor.get("Arm Motor");
        ArmServo = hardwareMap.servo.get("Arm Servo");
        Platform = hardwareMap.servo.get("Platform");
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
        ArmServo.setPosition(1);
        Gripper.setPosition(0.42);
        Platform.setPosition(0.1);
        
    }
    @Override
    public void loop() {
        
        //Assigning Values To Joysticks
        double drive = (-gamepad1.right_stick_x);
        double turn = (-gamepad1.right_stick_y);
        float StrafePower = (-gamepad1.left_stick_x);
        
        //Assigning Power Values
        double LeftPower = drive - turn;
        double RightPower = drive + turn;
        
        //Assigning Motors To Power Values
        FrontLeft.setPower(LeftPower);
        FrontRight.setPower(RightPower);
        BackLeft.setPower(LeftPower);
        BackRight.setPower(RightPower);
        Middle.setPower(StrafePower);
        
       /* if (gamepad2.dpad_up) {
            ArmMotor.setPower(1);
        } else if(gamepad2.dpad_down) {
            ArmMotor.setPower(-1);
        } else {
            ArmMotor.setPower(0);
        }*/
        
        float ArmPower = (gamepad2.left_stick_y);
        ArmMotor.setPower(ArmPower);
        
       
     while  (ArmSensor.isPressed()
        ) {
            ArmMotor.setPower(0);
        } 
        
        
        //Assigning Intake Triggers To Power Values
        float In = gamepad2.left_trigger;
        float Out = -gamepad2.right_trigger;
    
        //Assigning Motors To Power Values
        if (gamepad2.x){
            LeftIntake.setPower(-0.8);
            RightIntake.setPower(-0.8);
        }else if (gamepad2.b){
            LeftIntake.setPower(0);
            RightIntake.setPower(0);
        }else if (gamepad2.back) {
            RightIntake.setPower(1);
            LeftIntake.setPower(1);
        }
        if (gamepad1.left_bumper) {
        runtime.reset();
        while ((runtime.seconds() < 1.0)) {
            ArmMotor.setPower(-1);
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        }
        if (gamepad2.left_bumper){
            Gripper.setPosition(0.42);
        }else if (gamepad2.right_bumper){
            Gripper.setPosition(0.58);
            
        }
        if (gamepad2.dpad_up){
            ArmServo.setPosition(0);
            
        }else if (gamepad2.dpad_down){
            ArmServo.setPosition(1);
            
        }else if (gamepad2.dpad_left){
            ArmServo.setPosition(0.6);
            
        }
        
        if (gamepad1.dpad_left){
            Platform.setPosition(0.8);
            
        }else if (gamepad1.dpad_right){
            Platform.setPosition(0.1);
            
        }
    }
}
