package Java.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Blue Wheel Teleop", group = "Test")
public class BlueWheel_Teleop_Drive extends LinearOpMode {
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
        double Gear;
        double Dpaduplast;
        double Dpaddownlast;

        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");

        // Put initialization blocks here.
        // Create new IMU Parameters object.
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        BackRight.setDirection(DcMotorSimple.Direction.FORWARD);

        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
                telemetry.addData("Gear", Gear);

                double gpLSy = -gamepad1.left_stick_y;
                double gpLSx = gamepad1.left_stick_x;
                double gpRSx = -gamepad1.right_stick_x;
                telemetry.addData("LSy", gpLSy);
                telemetry.addData("LSx", gpLSx);
                telemetry.addData("RSx", gpRSx);
                telemetry.update();

                if (gamepad1.b) {
                    telemetry.addData("Button B", "Pressed");
                }

                FrontRight.setPower((gpLSy * Gear + gpRSx * Gear) - gpLSx * Gear);
                BackRight.setPower((gpLSy * Gear + gpRSx * Gear) + gpLSx * Gear);
                FrontLeft.setPower(gpLSy * Gear - gpRSx * Gear + gpLSx * Gear);
                BackLeft.setPower((gpLSy * Gear - gpRSx * Gear) - gpLSx * Gear);
                telemetry.update();
            }
        }
    }
}
