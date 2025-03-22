package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


@Autonomous(name = "Mark Auton02 SquareB Gyro Encoder", group = "Exercises")
// @Disabled
public class Mark_Auton02_SquareB_Gyro_Encoder extends LinearOpMode {
    private AndroidTextToSpeech androidTextToSpeech;
    private DcMotor FrontLeft, FrontRight, BackLeft, BackRight;
    private BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    private double globalAngle, power = .30, correction;

    /**
     * This function is executed when this Op Mode is selected from the Driver
     * Station.
     */
    @Override
    public void runOpMode() {
        // region Set Variables

        // region Distance measurements to encoder ticks
        // 1 full rotation is 13 inches
        // 537.6 tics is 1 full rotation

        double tFullRotation = 537.6;
        double iFullRotation = 13.0; // inches
        double tPerInch = tFullRotation / iFullRotation;
        double dInchesToSquareB = 106;
        int tForwardToSquareB = (int) Math.round(tPerInch * dInchesToSquareB);
        double dStrafeLeft01 = 26.0; // inches
        int tStrafeLeft01 = (int) Math.round(tPerInch * dStrafeLeft01);
        double dStrafeLeft02 = 5.0; // inches
        int tStrafeLeft02 = (int) Math.round(tPerInch * dStrafeLeft02);
        double dBackwardToPark = 20.0; // inches
        int tBackwardToPark = (int) Math.round(tPerInch * dBackwardToPark);
        // endregion Distance measurements to encoder ticks

        // Motor Power Variable
        double mP = 0.5;
        ElapsedTime ElapsedTime2;

        // Driving Encoder Ticks
        // tF - Ticks Forward
        // int tF = tForwardToSquareB;
        // tStrafeLeft01 - Ticks Left Strafe 1
        // int tStrafeLeft01 = 1100;
        // tStrafeLeft012 - Ticks Left Strafe 2
        // int tStrafeLeft02 = 3900 - tStrafeLeft01;
        // tB - Ticks Backward
        // int tBackwardToPark = 4000;
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
        ElapsedTime2 = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        // endregion Motor Initialization

        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        correction = checkDirection();
        // region Encoder Initialization
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Move forward 3000 ticks 79.5 in
        FrontLeft.setTargetPosition(tForwardToSquareB);
        BackLeft.setTargetPosition(tForwardToSquareB);
        FrontRight.setTargetPosition(tForwardToSquareB);
        BackRight.setTargetPosition(tForwardToSquareB);
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // endregion Encoder Initialization

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C
        // port
        // on a Core Device Interface Module, configured to be a sensor of type
        // "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        // Report the initialization to the Driver Station.
        telemetry.addData("Status", "IMU initialized, calibration started.");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            telemetry.addData("If calibration ",
                    "doesn't complete after 3 seconds, move through 90 degree pitch, roll and yaw motions until calibration complete ");
            telemetry.update();
            sleep(50);
            idle();
        }

        // Report calibration complete to Driver Station.
        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.addData("Status", "Calibration Complete");
        telemetry.addData("Action needed:", "Please press the start triangle");
        telemetry.update();

        waitForStart();

        // region move forward1
        // Run forward until all motors have achieved their ticks
        androidTextToSpeech.speak("I will move forward to square B");

        // Use gyro to drive in a straight line.
        correction = checkDirection();

        telemetry.addData("1 imu heading", lastAngles.firstAngle);
        telemetry.addData("2 global heading", globalAngle);
        telemetry.addData("3 correction", correction);
        telemetry.update();

        FrontRight.setPower(mP - correction);
        FrontLeft.setPower(mP + correction);
        BackRight.setPower(mP - correction);
        BackLeft.setPower(mP + correction);
        // endregion move forward1

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

        // region strafe left

        // Move Robot: Strafe Left
        androidTextToSpeech.speak("Strafe Left");
        // Set TargetPosition and ticks
        FrontLeft.setTargetPosition(-tStrafeLeft01);
        BackLeft.setTargetPosition(tStrafeLeft01);
        FrontRight.setTargetPosition(tStrafeLeft01);
        BackRight.setTargetPosition(-tStrafeLeft01);
        // Set Mode to RUN_TO_POSITION for all motors
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // endregion strafe left

        while (!(!FrontLeft.isBusy() && !BackLeft.isBusy() && !FrontRight.isBusy() && !BackRight.isBusy())) {
        }

        // Add code to drop wobble
        // While dropping wobble using servo do not move motors

        // Wait two seconds
        sleep(2000);

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

        // region strafe left

        // Move Robot: Strafe Left
        androidTextToSpeech.speak("Strafe Left");
        // Set TargetPosition and ticks
        FrontLeft.setTargetPosition(-tStrafeLeft02);
        BackLeft.setTargetPosition(tStrafeLeft02);
        FrontRight.setTargetPosition(tStrafeLeft02);
        BackRight.setTargetPosition(-tStrafeLeft02);
        // Set Mode to RUN_TO_POSITION for all motors
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // endregion strafe left

        while (!(!FrontLeft.isBusy() && !BackLeft.isBusy() && !FrontRight.isBusy() && !BackRight.isBusy())) {
        }
        // region reset encode and reverse direction
        // Reset encoders reverse motors right set
        // ticks tell motors to move
        //
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // endregion reset encode and reverse direction

        // region Drive Backward Encoder Count
        // All Motors

        androidTextToSpeech.speak("I will move backward");
        FrontLeft.setTargetPosition(-tBackwardToPark);
        BackLeft.setTargetPosition(-tBackwardToPark);
        FrontRight.setTargetPosition(-tBackwardToPark);
        BackRight.setTargetPosition(-tBackwardToPark);
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // endregion Drive Backward Encoder Count

        while (!(!FrontLeft.isBusy() && !BackLeft.isBusy() && !FrontRight.isBusy() && !BackRight.isBusy())) {
        }

        rotate(-90, mP);

        androidTextToSpeech.speak("I am done");
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * 
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for
        // heading angle.
        // We have to process the angle because the imu works in euler angles so the Z
        // axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation
        // passes
        // 180 degrees. We detect this transition and track the total cumulative angle
        // of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction
     * value.
     * 
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection() {
        // The gain value determines how sensitive the correction is to direction
        // changes.
        // You will have to experiment with your robot to get small smooth direction
        // changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0; // no adjustment.
        else
            correction = -angle; // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more
     * than 180 degrees.
     * 
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power) {
        double leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when
        // rotating
        // clockwise (right).

        if (degrees < 0) { // turn right.
            leftPower = power;
            rightPower = -power;
        } else if (degrees > 0) { // turn left.
            leftPower = -power;
            rightPower = power;
        } else
            return;

        // set power to rotate.
        FrontLeft.setPower(leftPower);
        FrontRight.setPower(rightPower);
        BackLeft.setPower(leftPower);
        BackRight.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {
            }

            while (opModeIsActive() && getAngle() > degrees) {
            }
        } else // left turn.
            while (opModeIsActive() && getAngle() < degrees) {
            }

        // turn the motors off.
        FrontRight.setPower(0);
        FrontLeft.setPower(0);
        BackRight.setPower(0);
        BackLeft.setPower(0);

        // wait for rotation to stop.
        sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }
}
