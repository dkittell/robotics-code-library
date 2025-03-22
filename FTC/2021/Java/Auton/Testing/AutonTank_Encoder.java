package Java.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech;

@Autonomous(name = "AutonTank_Encoder (Java)", group = "")
public class AutonTank_Encoder extends LinearOpMode {
	private AndroidTextToSpeech androidTextToSpeech;
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
		// region Set Variables

		// region Distance measurements to encoder encoder counts

		// 1 full rotation is ? inches
		// 1120 Encoder Counts is 1 full rotation
		// https://www.andymark.com/products/neverest-classic-40-gearmotor
		double tFullRotation = 1120; // Elon M Robot
		double iFullRotation = 12.75; // Inches - Elon M Robot

		// 1 full rotation is 13 inches
		// 537.6 Encoder Counts is 1 full rotation
		// https://www.andymark.com/products/neverest-orbital-20-gearmotor
		// double tFullRotation = 537.6; // Mark Robot
		// double iFullRotation = 13.0; // Inches - Mark Robot
		double tPerInch = tFullRotation / iFullRotation;
		double dInchesStrafeRightToSquareA = 77;
		int tRightStrafeToSquareA = (int) Math.round(tPerInch * dInchesStrafeRightToSquareA);
		double dInchesForward01 = 19.0; // inches
		int tForward01 = (int) Math.round(tPerInch * dInchesForward01);
		double dInchesForward02 = 35.0; // inches
		int tForward02 = (int) Math.round(tPerInch * dInchesForward02);
		double dInchesBackward01 = 25.0; // inches
		int tBackward01 = (int) Math.round(tPerInch * dInchesBackward01);
		double dInchesBackward02 = 13.0; // inches
		int tBackward02 = (int) Math.round(tPerInch * dInchesBackward02);
		// endregion Distance measurements to encoder Encoder Counts

		// Motor Power Variable
		double mP = 0.5;

		// Driving Encoder Encoder Counts
		// ecrs - Encoder Counts Right Strafe
		// int ecrs = ecRightStrafeToSquareB;
		// ecStrafeLeftToPark - Encoder Counts Left Strafe To Park
		// int ecStrafeLeftToPark = 5376;
		// ecF - Encoder Counts Forward
		// int ecF = ecForward
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
		// endregion Motor Initialization

		// region Encoder Initialization
		FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		// Move forward 3000 Encoder Counts 79.5 in
		FrontLeft.setTargetPosition(1120);
		BackLeft.setTargetPosition(1120);
		FrontRight.setTargetPosition(1120);
		BackRight.setTargetPosition(1120);
		FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		// endregion Encoder Initialization

		telemetry.addData("Action needed", "Please press the start triangle");
		telemetry.addData("Encoder Counts To Move", tForward01);
		telemetry.update();
		waitForStart();

		// region move forward1
		// Run forward until all motors have achieved their Encoder Counts
		androidTextToSpeech.speak("I will move forward");
		FrontLeft.setPower(mP);
		BackLeft.setPower(mP);
		FrontRight.setPower(mP);
		BackRight.setPower(mP);
		// endregion move forward1

		while (!(!FrontLeft.isBusy() && !BackLeft.isBusy() && !FrontRight.isBusy() && !BackRight.isBusy())) {
		}

		FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		sleep(2000);

		while (!(!FrontLeft.isBusy() && !BackLeft.isBusy() && !FrontRight.isBusy() && !BackRight.isBusy())) {
		}

		// region Drive Backward Encoder Count
		// All Motors

		androidTextToSpeech.speak("I am done");
		// endregion Drive Backward Encoder Count
		FrontLeft.setPower(0);
		BackLeft.setPower(0);
		FrontRight.setPower(0);
		BackRight.setPower(0);
	}
}
