package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Competition 2021 Teleop", group = "Competition")
public class Competition_2021_Teleop extends LinearOpMode {
    private DcMotor mFrontRight;
    private DcMotor mBackRight;
    private DcMotor mFrontLeft;
    private DcMotor mBackLeft;
    private DcMotor mPivot;
    private DcMotor mArm;
    private DcMotor mDuck;
    private TouchSensor tsArm;
    private TouchSensor tsPivot;
    private CRServo sIntake;
     
    /**
     * This function is executed when this Op Mode is selected from the Driver
     * Station.
     */
    @Override
    public void runOpMode() {
        double Gear;
        double Dpaduplast;
        double Dpaddownlast;

        mPivot = hardwareMap.get(DcMotor.class, "mPivot");
        mArm = hardwareMap.get(DcMotor.class, "mArm");
        tsArm = hardwareMap.get(TouchSensor.class, "tsArm");
        tsPivot = hardwareMap.get(TouchSensor.class, "tsPivot");
        sIntake = hardwareMap.get(CRServo.class, "sIntake");

        mFrontLeft = hardwareMap.get(DcMotor.class, "mFrontLeft");
        mBackLeft = hardwareMap.get(DcMotor.class, "mBackLeft");
        mFrontRight = hardwareMap.get(DcMotor.class, "mFrontRight");
        mBackRight = hardwareMap.get(DcMotor.class, "mBackRight");
        mDuck = hardwareMap.get(DcMotor.class, "mDuck");

        // Put initialization blocks here.
        // Create new IMU Parameters object.
        mFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        mBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        mFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        mBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        mFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        mBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        mFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        mBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        mPivot.setDirection(DcMotorSimple.Direction.FORWARD);
        mArm.setDirection(DcMotorSimple.Direction.FORWARD);
        mPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Gear = 0.1;
        Dpaduplast = 0;
        Dpaddownlast = 0;

        telemetry.addData("Action needed", "Please press the start triangle");
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.

                //region Operator Controller
                // region Arm Extension
                // mPivot.setPower(gamepad2.left_stick_y);
                if (gamepad2.right_stick_y < 0) {
                    if (tsArm.isPressed()) {
                        mArm.setPower(0);
                    } else {
                        mArm.setPower(gamepad2.right_stick_y);
                    }
                    telemetry.update();
                } else {
                    mArm.setPower(gamepad2.right_stick_y);
                }
                 // endregion Arm Extension

                 //region Intake Servo
                if (gamepad2.right_trigger > 0.5) {
                    sIntake.setPower(gamepad2.right_trigger);
                } else if (gamepad2.left_trigger > 0.5) {
                    sIntake.setPower(-gamepad2.left_trigger);
                } else {
                    sIntake.setPower(0);
                }
                //endregion Intake Servo

                //region Pivot - Arm Up/Down
                if (gamepad2.left_stick_y > 0) {
                    if (tsPivot.isPressed()) {
                        mPivot.setPower(0);
                      //   mPivot.setPower(gamepad2.left_stick_y);
                    } else {
                        mPivot.setPower(gamepad2.left_stick_y);
                       //  mPivot.setPower(0);
                    }
                    telemetry.update();
                } else {
                    mPivot.setPower(gamepad2.left_stick_y);
                }
                //endregion Pivot - Arm Up/Down
                
                telemetry.update();
                //endregion Operator Controller

                // Arcade style meccanum drive
                if (Dpaduplast == 0) {
                    if (Gear < 0.89

                    ) {
                        if (gamepad1.dpad_up) {
                            Dpaduplast = 1;
                            Gear += 0.1;
                        }
                    }
                } else if (!gamepad1.dpad_up) {
                    Dpaduplast = 0;
                }
                if (Dpaddownlast == 0) {
                    if (Gear > 0.11) {
                        if (gamepad1.dpad_down) {
                            Gear += -0.1;
                            Dpaddownlast = 1;
                        }
                    }
                } else if (!gamepad1.dpad_down) {
                    Dpaddownlast = 0;
                }

                telemetry.addData("Time Passed", getRuntime());
                telemetry.addData("Gear", (Gear * 10));

                double gpLSy = -gamepad1.left_stick_y;
                double gpLSx = -gamepad1.left_stick_x;
                double gpRSx = gamepad1.right_stick_x;
                telemetry.addData("LSy", gpLSy);
                telemetry.addData("LSx", gpLSx);
                telemetry.addData("RSx", gpRSx);
                telemetry.update();                   

                //region Driver Controller
                double mFRP = (-gpLSy * Gear + gpRSx * Gear) - gpLSx * Gear;
                double mBRP = (-gpLSy * Gear + gpRSx * Gear) + gpLSx * Gear;

                // For some reason the left side is stronger than the right so we slow it down a
                // bit.
                double mFLP = ((-gpLSy * Gear - gpRSx * Gear) + gpLSx * Gear) * 0.8;
                double mBLP = ((-gpLSy * Gear - gpRSx * Gear) - gpLSx * Gear) * 0.8;

                mFrontLeft.setPower(mFLP);
                mBackLeft.setPower(mBLP);
                mFrontRight.setPower(mFRP);
                mBackRight.setPower(mBRP);   

                //region Duck Motor - Driver Controller
                if (gamepad1.left_bumper || gamepad2.left_bumper) {
                    mDuck.setPower(-1);
                } else {
                    mDuck.setPower(0);
                }

                if (gamepad1.right_bumper || gamepad2.right_bumper) {
                    mDuck.setPower(1);
                } else {
                    mDuck.setPower(0);
                }
                //endregion Duck Motor - Driver Controller
                //endregion Driver Controller
               
                telemetry.update();
            }
        }
    }
}
