package Java.Auton.Competition;

//region Imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//endregion Imports

@Autonomous(name = "Red Duck", group = "Competition", preselectTeleOp = "Competition 2021 Teleop")
public class Auton_Red_Duck extends LinearOpMode {

    // region Constants
    // region Motor Definitions
    private DcMotor mFrontRight;
    private DcMotor mBackRight;
    private DcMotor mFrontLeft;
    private DcMotor mBackLeft;

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

        // endregion Set Values To contants

        telemetry.addData("Action needed", "Please press the start triangle");
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.

            // telemetry.speak("Forward", null, null);
            // DriveRobotEncoder(1000, 0.5, 0, 5);

            // telemetry.speak("Forward", null, null);
            // DriveRobot(1000, 0.5, 0);

            // telemetry.speak("Backward", null, null);
            // DriveRobot(1000, 0.5, 1);

            // Strafe Left
            mFrontLeft.setPower(-0.8);
            mBackLeft.setPower(0.8);
            mFrontRight.setPower(0.8);
            mBackRight.setPower(-0.8);

            sleep(250);

            mFrontLeft.setPower(0);
            mBackLeft.setPower(0);
            mFrontRight.setPower(0);
            mBackRight.setPower(0);

            // telemetry.speak("Strafe Left", null, null);
            // DriveRobotEncoder(1000, 0.5, 4, 15);

            telemetry.speak("Backward", null, null);
            mFrontLeft.setPower(-0.5);
            mBackLeft.setPower(-0.5);
            mFrontRight.setPower(-0.5);
            mBackRight.setPower(-0.5);

            sleep(1450);

            mFrontLeft.setPower(0);
            mBackLeft.setPower(0);
            mFrontRight.setPower(0);
            mBackRight.setPower(0);

            // telemetry.speak("Strafe right", null, null);
            // DriveRobot(1000, 0.5, 5);

            // telemetry.speak("Forward", null, null);
            // DriveRobot(1000, 0.5, 0);

            // telemetry.speak("Turn left", null, null);
            // DriveRobot(1000, 0.5, 2);

            // telemetry.speak("Turn right", null, null);
            // DriveRobot(1000, 0.5, 3);

            mDuck.setPower(-0.6);

            sleep(3500);

            // telemetry.speak("Strafe Left", null, null);
            // DriveRobotEncoder(1000, 0.5, 4, 15);

            // Strafe Left
            mFrontLeft.setPower(-0.8);
            mBackLeft.setPower(0.8);
            mFrontRight.setPower(0.8);
            mBackRight.setPower(-0.8);

            sleep(1250);

            mFrontLeft.setPower(0);
            mBackLeft.setPower(0);
            mFrontRight.setPower(0);
            mBackRight.setPower(0);

            sleep(100);
            mFrontLeft.setPower(-0.5);
            mBackLeft.setPower(-0.5);
            mFrontRight.setPower(-0.5);
            mBackRight.setPower(-0.5);

            sleep(200);

            mFrontLeft.setPower(0);
            mBackLeft.setPower(0);
            mFrontRight.setPower(0);
            mBackRight.setPower(0);

            telemetry.update();
        }
    }
}
