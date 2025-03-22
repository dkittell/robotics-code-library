package org.firstinspires.ftc.teamcode;

//region Imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//endregion Imports

@Autonomous(name = "Blue Preloaded Block", group = "Competition", preselectTeleOp = "Competition 2021 Teleop")
// @Disabled
public class Auton_Blue_Preloaded_Block extends LinearOpMode {

    // region Constants
    // region Motor Definitions
    private DcMotor mFrontRight;
    private DcMotor mBackRight;
    private DcMotor mFrontLeft;
    private DcMotor mBackLeft;

    private CRServo sIntake;
    private DcMotor mPivot;
    private DcMotor mArm;
    private TouchSensor tsArm;
    private TouchSensor tsPivot;
    private DcMotor mDuck;
    // endregion Motor Definitions

    // endregion Constants

    // region functions

    public void DriveRobot(int ms, double mp, int nDirection) {
        switch (nDirection) {
            case 0:
                // Drive Forward
                mFrontLeft.setPower(mp);
                mBackLeft.setPower(mp);
                mFrontRight.setPower(mp);
                mBackRight.setPower(mp);
                break;
            case 1:
                // Drive Reverse
                mFrontLeft.setPower(-mp);
                mBackLeft.setPower(-mp);
                mFrontRight.setPower(-mp);
                mBackRight.setPower(-mp);
                break;
            case 2:
                // Turn Left
                mFrontLeft.setPower(-mp);
                mBackLeft.setPower(-mp);
                mFrontRight.setPower(mp);
                mBackRight.setPower(mp);
                break;
            case 3:
                // Turn Right
                mFrontLeft.setPower(mp);
                mBackLeft.setPower(mp);
                mFrontRight.setPower(-mp);
                mBackRight.setPower(-mp);
                break;
            case 4:
                // Strafe Left
                mFrontLeft.setPower(-mp);
                mBackLeft.setPower(mp);
                mFrontRight.setPower(mp);
                mBackRight.setPower(-mp);
                break;
            case 5:
                // Strafe Right
                mFrontLeft.setPower(mp);
                mBackLeft.setPower(-mp);
                mFrontRight.setPower(-mp);
                mBackRight.setPower(mp);
                break;
            default:
                mFrontLeft.setPower(0);
                mBackLeft.setPower(0);
                mFrontRight.setPower(0);
                mBackRight.setPower(0);
                break;
        }
        sleep(ms);
    }

    public void DriveRobotEncoder(int ms, double mp, int nDirection, double dInch) {

        int ecFR;
        double inFR;
        int ecPerInch;

        ecFR = 1120;
        inFR = 12.75;
        ecPerInch = ecFR / (int) inFR;

        int Ec = ecPerInch * (int) dInch;

        mFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        switch (nDirection) {
            case 0:
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
                while (!(!mFrontLeft.isBusy() && !mBackLeft.isBusy() && !mFrontRight.isBusy()
                        && !mBackRight.isBusy())) {
                }
                // Encoder Count Drive Forward - End

                break;
            case 1:
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
                while (!(!mFrontLeft.isBusy() && !mBackLeft.isBusy() && !mFrontRight.isBusy() && !mBackRight.isBusy()))
                    break;
            case 2:
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
                while (!(!mFrontLeft.isBusy() && !mBackLeft.isBusy() && !mFrontRight.isBusy() && !mBackRight.isBusy()))
                    break;
            case 3:
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
                while (!(!mFrontLeft.isBusy() && !mBackLeft.isBusy() && !mFrontRight.isBusy() && !mBackRight.isBusy()))
                    break;

            case 4:
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
                while (!(!mFrontLeft.isBusy() && !mBackLeft.isBusy() && !mFrontRight.isBusy() && !mBackRight.isBusy()))
                    break;
            case 5:
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
                while (!(!mFrontLeft.isBusy() && !mBackLeft.isBusy() && !mFrontRight.isBusy()
                        && !mBackRight.isBusy())) {
                }

                break;
            default:

                mFrontLeft.setPower(0);
                mBackLeft.setPower(0);
                mFrontRight.setPower(0);
                mBackRight.setPower(0);
                break;
        }

        sleep(ms);
        telemetry.addData("Ec", Ec);
        telemetry.update();
    }
    // endregion functions

    /**
     * This function is executed when this Op Mode is selected from the Driver
     * Station.
     */
    @Override
    public void runOpMode() {

        // region Set Values To contants
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
        mFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        mFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        mBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        mBackRight.setDirection(DcMotorSimple.Direction.FORWARD);

        mFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mPivot.setDirection(DcMotorSimple.Direction.FORWARD);
        mArm.setDirection(DcMotorSimple.Direction.FORWARD);
        mPivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // endregion Set Values To contants

        telemetry.addData("Action needed", "Please press the start triangle");
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.

            // telemetry.speak("Forward", null, null);
            // DriveRobotEncoder(1000, 0.5, 0, 41);

            telemetry.speak("Pivot", null, null);
            if (tsArm.isPressed()) {
                mPivot.setPower(0);
            } else {

                mPivot.setPower(-1);
                // mArm.setPower(-.5);
                sleep(2500);
                mPivot.setPower(0);
            }

            // telemetry.speak("Arm", null, null);
            // if (tsArm.isPressed()) {
            // mArm.setPower(0);
            // } else {
            // mArm.setPower(-1);
            // sleep(1000);
            // }
            // mArm.setPower(0);

            telemetry.speak("Arm", null, null);
            if (tsArm.isPressed()) {
                mArm.setPower(0);
            } else {

                mArm.setPower(-1);
                sleep(1000);
            }
            mArm.setPower(0);

            telemetry.speak("Forward", null, null);
            mFrontLeft.setPower(1 * 0.9);
            mBackLeft.setPower(1 * 0.9);
            mFrontRight.setPower(1);
            mBackRight.setPower(1);
            sleep(700);
            sIntake.setPower(0);
            mFrontLeft.setPower(0);
            mBackLeft.setPower(0);
            mFrontRight.setPower(0);
            mBackRight.setPower(0);

            telemetry.speak("Intake", null, null);
            sIntake.setPower(-1);
            sleep(4000);
            sIntake.setPower(0);

            telemetry.speak("Backward", null, null);
            mFrontLeft.setPower(-0.8 * 0.9);
            mBackLeft.setPower(-0.8 * 0.9);
            mFrontRight.setPower(-0.8);
            mBackRight.setPower(-0.8);
            sleep(700);
            mFrontLeft.setPower(0);
            mBackLeft.setPower(0);
            mFrontRight.setPower(0);
            mBackRight.setPower(0);

            telemetry.speak("Arm", null, null);
            if (tsArm.isPressed()) {
                mArm.setPower(1);
                sleep(1000);
            } else {
                mArm.setPower(0);

            }
            mArm.setPower(0);

            // telemetry.speak("Arm", null, null);
            // if (tsArm.isPressed()) {
            // mArm.setPower(0);
            // } else {
            // mArm.setPower(1);
            // sleep(2000);
            // }
            // mArm.setPower(0);

            telemetry.speak("Pivot", null, null);
            if (tsArm.isPressed()) {

                mPivot.setPower(0);
            } else {
                mPivot.setPower(1);
                // mArm.setPower(-.5);
                sleep(1900);
                mPivot.setPower(0);
            }

            telemetry.speak("Strafe Right", null, null);
            mFrontLeft.setPower(0.8);
            mBackLeft.setPower(-0.8 * 0.8);
            mFrontRight.setPower(-0.8 * 0.8);
            mBackRight.setPower(0.8);
            sleep(3500);
            mFrontLeft.setPower(0);
            mBackLeft.setPower(0);
            mFrontRight.setPower(0);
            mBackRight.setPower(0);

            telemetry.speak("Backward", null, null);
            mFrontLeft.setPower(-0.8 * 0.9);
            mBackLeft.setPower(-0.8 * 0.9);
            mFrontRight.setPower(-0.8);
            mBackRight.setPower(-0.8);
            sleep(300);
            mFrontLeft.setPower(0);
            mBackLeft.setPower(0);
            mFrontRight.setPower(0);
            mBackRight.setPower(0);

            mDuck.setPower(0.6);
            sleep(2850);

            telemetry.speak("Forward", null, null);
            mFrontLeft.setPower(1 * 0.8);
            mBackLeft.setPower(1 * 0.8);
            mFrontRight.setPower(1);
            mBackRight.setPower(1);
            sleep(850);
            sIntake.setPower(0);
            mFrontLeft.setPower(0);
            mBackLeft.setPower(0);
            mFrontRight.setPower(0);
            mBackRight.setPower(0);

            telemetry.update();
        }
    }
}
