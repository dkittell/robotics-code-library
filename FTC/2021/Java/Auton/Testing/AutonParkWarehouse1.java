package Java.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "AutonParkWarehouse1 (Blocks to Java)")
public class AutonParkWarehouse1 extends LinearOpMode {

    private DcMotor FrontRight;
    private DcMotor BackRight;
    private DcMotor FrontLeft;
    private DcMotor BackLeft;

    /**
     * This function is executed when this Op Mode is selected from the Driver
     * Station.
     */
    @Override
    public void runOpMode() {
        int ecForward;
        int ecStrafeR;
        double mP;

        FrontRight = hardwareMap.get(DcMotor.class, "Front Right");
        BackRight = hardwareMap.get(DcMotor.class, "Back Right");
        FrontLeft = hardwareMap.get(DcMotor.class, "Front Left");
        BackLeft = hardwareMap.get(DcMotor.class, "Back Left");

        // Put initialization blocks here.
        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int ecfr = 1120;
        int infr = 13;
        int ecperinch = ecfr / (int) infr;

        ecForward = ecperinch * 2;
        ecStrafeR = ecperinch * 30;
        mP = 0.5;
        waitForStart();
      if ( opModeIsActive() ) {
            // Put run blocks here.
            FrontLeft.setPower(mP);
            BackLeft.setPower(mP);
            FrontRight.setPower(mP);
            BackRight.setPower(mP);
            FrontLeft.setTargetPosition(ecForward);
            BackLeft.setTargetPosition(ecForward);
            FrontRight.setTargetPosition(ecForward);
            BackRight.setTargetPosition(ecForward);
            FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(1000);
            FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FrontLeft.setPower(mP);
            BackLeft.setPower(mP);
            FrontLeft.setPower(mP);
            BackLeft.setPower(mP);
            BackLeft.setTargetPosition(-ecStrafeR);
            BackRight.setTargetPosition(ecStrafeR);
            FrontLeft.setTargetPosition(ecStrafeR);
            FrontRight.setTargetPosition(-ecStrafeR);
            BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(3000);
        }
    }
}
