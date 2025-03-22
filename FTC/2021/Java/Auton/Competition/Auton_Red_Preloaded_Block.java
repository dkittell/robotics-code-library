package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name = "Red Preloaded Block", group = "Competition", preselectTeleOp = "Competition 2021 Teleop")
public class Auton_Red_Preloaded_Block extends LinearOpMode {

    private BNO055IMU gyro;

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

    /**
     * Function that becomes true when gyro is calibrated and
     * reports calibration status to Driver Station in the meantime.
     */
    private boolean IMU_Calibrated() {
        telemetry.addData("IMU Calibration Status", gyro.getCalibrationStatus());
        telemetry.addData("Gyro Calibrated", gyro.isGyroCalibrated() ? "True" : "False");
        telemetry.addData("System Status", gyro.getSystemStatus().toString());
        return gyro.isGyroCalibrated();
    }

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
        BNO055IMU.Parameters IMU_Parameters;
        double Left_Power;
        double Right_Power;
        float Yaw_Angle = 0;

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
        gyro = hardwareMap.get(BNO055IMU.class, "gyro");

        // This op mode uses the REV Hub's built-in gyro to
        // to allow a robot to move straight forward and then
        // make a right turn.
        // The op mode assume you have
        // (1) Connected two motors to the expansion
        // hub.
        // (2) Created a config file that
        // (a) names the motors "left-motor" and
        // "right-motor"
        // (b) configures the imu on I2C bus 0 port 0
        // as a REV Expansion Hub IMU
        // with the name "imu".
        // Setup so motors will brake the wheels
        // when motor power is set to zero.
        // Reverse direction of one motor so robot moves
        // forward rather than spinning in place.
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

        // Create an IMU parameters object.
        IMU_Parameters = new BNO055IMU.Parameters();
        // Set the IMU sensor mode to IMU. This mode uses
        // the IMU accelerometer and gyroscope data to
        // calculate the relative orientation of hub and
        // therefore the robot.
        IMU_Parameters.mode = BNO055IMU.SensorMode.IMU;
        // Intialize the IMU using parameters object.
        gyro.initialize(IMU_Parameters);
        // Report the initialization to the Driver Station.
        telemetry.addData("Status", "IMU initialized");
        telemetry.update();
        // Wait one second to ensure the IMU is ready.
        sleep(1000);
        // Loop until IMU has been calibrated.
        while (!IMU_Calibrated()) {
            telemetry.addData("If calibration ",
                    "doesn't complete after 3 seconds, move through 90 degree pitch, roll and yaw motions until calibration complete ");
            telemetry.update();
            // Wait one second before checking calibration
            // status again.
            sleep(1000);
        }
        // Report calibration complete to Driver Station.
        telemetry.addData("Status", "Calibration Complete");
        telemetry.addData("Action needed", "Please press the start triangle");
        telemetry.update();
        // Wait for Start to be pressed on Driver Station.
        waitForStart();

        if (opModeIsActive()) {

            telemetry.speak("Pivot", null, null);
            if (tsArm.isPressed()) {
                mPivot.setPower(0);
            } else {

                mPivot.setPower(-1);
                // mArm.setPower(-.5);
                sleep(2500);
                mPivot.setPower(0);
            }

            telemetry.speak("Arm", null, null);
            if (tsArm.isPressed()) {
                mArm.setPower(0);
            } else {

                mArm.setPower(-1);
                sleep(1000);
            }
            mArm.setPower(0);

            // Initialize motor power variables to 30%.
            telemetry.speak("Forward", null, null);
            mFrontLeft.setPower(1 * 0.9);
            mBackLeft.setPower(1 * 0.9);
            mFrontRight.setPower(1);
            mBackRight.setPower(1);
            sleep(600);
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
            sleep(500);
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

            telemetry.speak("Pivot", null, null);
            if (tsArm.isPressed()) {

                mPivot.setPower(0);
            } else {
                mPivot.setPower(1);
                // mArm.setPower(-.5);
                sleep(1900);
                mPivot.setPower(0);
            }
            // Save gyro's yaw angle
            Yaw_Angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ,
                    AngleUnit.DEGREES).thirdAngle;
            // Report yaw orientation to Driver Station.
            telemetry.addData("Yaw value", Yaw_Angle);
            // If the robot is moving straight ahead the
            // yaw value will be close to zero. If it's not, we
            // need to adjust the motor powers to adjust heading.
            // If robot yaws right or left by 5 or more,
            // adjust motor power variables to compensation.
            if (Yaw_Angle < -5) {
                // Turn left
                Left_Power = 0.25;
                Right_Power = 0.35;
            } else if (Yaw_Angle > 5) {
                // Turn right.
                Left_Power = 0.35;
                Right_Power = 0.25;
            } else {
                // Continue straight
                Left_Power = 0.3;
                Right_Power = 0.3;
            }
            // Report the new power levels to the Driver Station.
            telemetry.addData("Left Motor Power", Left_Power);
            telemetry.addData("Right Motor Power", Right_Power);
            // Update the motors to the new power levels.
            mFrontLeft.setPower(Left_Power);
            mFrontRight.setPower(Right_Power);
            mBackLeft.setPower(Left_Power);
            mBackRight.setPower(Right_Power);
            telemetry.update();
            // Wait 1/5 second before checking again.
            sleep(200);

            // Now let's execute a right turn using power
            // levels that will cause a turn in place.
            mFrontLeft.setPower(0.4);
            mFrontRight.setPower(-0.4);
            mBackLeft.setPower(0.4);
            mBackRight.setPower(-0.4);
            // Continue until robot yaws right by 90 degrees
            // or stop is pressed on Driver Station.
            while (!(Yaw_Angle <= -90 || isStopRequested())) {
                // Update Yaw-Angle variable with current yaw.
                Yaw_Angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ,
                        AngleUnit.DEGREES).thirdAngle;
                // Report yaw orientation to Driver Station.
                telemetry.addData("Yaw value", Yaw_Angle);
                telemetry.update();
            }
            mFrontLeft.setPower(-0.8);
            mFrontRight.setPower(-0.8);
            mBackLeft.setPower(-0.8);
            mBackRight.setPower(-0.8);
            telemetry.update();
            sleep(2000);
            // We're done. Turn off motors
            mFrontLeft.setPower(0);
            mFrontRight.setPower(0);
            mBackLeft.setPower(0);
            mBackRight.setPower(0);
            // Pause so final yaw is displayed.
            // Strafe Right
            mFrontLeft.setPower(0.8);
            mBackLeft.setPower(-0.8 * 0.9);
            mFrontRight.setPower(-0.8 * 0.9);
            mBackRight.setPower(0.8);
            sleep(700);
            mFrontLeft.setPower(0);
            mBackLeft.setPower(0);
            mFrontRight.setPower(0);
            mBackRight.setPower(0);

            mDuck.setPower(-0.6);
            sleep(3500);

            mFrontLeft.setPower(-1);
            mBackLeft.setPower(1 * 0.9);
            mFrontRight.setPower(1 * 0.9);
            mBackRight.setPower(-1);

            sleep(970);

            telemetry.speak("Backward", null, null);
            mFrontLeft.setPower(-0.8 * 0.9);
            mBackLeft.setPower(-0.8 * 0.9);
            mFrontRight.setPower(-0.8);
            mBackRight.setPower(-0.8);
            sleep(250);

            mFrontLeft.setPower(0);
            mBackLeft.setPower(0);
            mFrontRight.setPower(0);
            mBackRight.setPower(0);

            telemetry.update();
        }
    }
}
