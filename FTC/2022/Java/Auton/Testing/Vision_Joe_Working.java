package org.firstinspires.ftc.teamcode.Competition.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.Tfod;

@Autonomous(name = "VisionJoe")
// @Disabled
public class Vision_Joe_Working extends LinearOpMode {

  private AndroidTextToSpeech androidTextToSpeech;
  private VuforiaCurrentGame vuforiaPOWERPLAY;
  private DcMotor mBackLeft;
  private DcMotor mFrontLeft;
  private Tfod tfod;
  private DcMotor Lights;
  private CRServo sGrabOne;
  private CRServo sGrabTwo;

  boolean isGreenDetected;
  Recognition recognition;
  String Green;
  boolean isBlueDetected;
  String Blue;
  boolean isRedDetected;
  String Red;

  /**
   * Describe this function...
   */
  private void displayInfo(int i) {
    // Display the location of the top left corner
    // of the detection boundary for the recognition
    telemetry.addData(
      "Label: " +
      recognition.getLabel() +
      ", Confidence: " +
      recognition.getConfidence(),
      "X: " +
      Math.round(
        JavaUtil.averageOfList(
          JavaUtil.createListWith(
            Double.parseDouble(JavaUtil.formatNumber(recognition.getLeft(), 0)),
            Double.parseDouble(JavaUtil.formatNumber(recognition.getRight(), 0))
          )
        )
      ) +
      ", Y: " +
      Math.round(
        JavaUtil.averageOfList(
          JavaUtil.createListWith(
            Double.parseDouble(JavaUtil.formatNumber(recognition.getTop(), 0)),
            Double.parseDouble(
              JavaUtil.formatNumber(recognition.getBottom(), 0)
            )
          )
        )
      )
    );
    if (recognition.getLabel().equals(Green)) {
      isGreenDetected = true;
      telemetry.addData("Object Detected", "Green");
    } else {
      isGreenDetected = false;
    }
    if (recognition.getLabel().equals(Blue)) {
      isBlueDetected = true;
      telemetry.addData("Object Detected", "Blue");
    } else {
      isBlueDetected = false;
    }
    if (recognition.getLabel().equals(Red)) {
      isRedDetected = true;
      telemetry.addData("Object Detected", "Red");
    } else {
      isRedDetected = false;
    }
  }

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    List<Recognition> recognitions;
    int index;

    androidTextToSpeech = new AndroidTextToSpeech();
    vuforiaPOWERPLAY = new VuforiaCurrentGame();
    mBackLeft = hardwareMap.get(DcMotor.class, "mBackLeft");
    mFrontLeft = hardwareMap.get(DcMotor.class, "mFrontLeft");
    tfod = new Tfod();
    Lights = hardwareMap.get(DcMotor.class, "mLights");
    sGrabOne = hardwareMap.get(CRServo.class, "sGrabOne");
    sGrabTwo = hardwareMap.get(CRServo.class, "sGrabTwo");

    androidTextToSpeech.initialize();
    androidTextToSpeech.setLanguage("en");
    // Sample TFOD Op Mode using a Custom Model
    // Initialize Vuforia to provide TFOD with camera
    // images.
    // The following block uses the device's back camera.
    // The following block uses a webcam.
    vuforiaPOWERPLAY.initialize(
      "", // vuforiaLicenseKey
      hardwareMap.get(WebcamName.class, "wcVision"), // cameraName
      "", // webcamCalibrationFilename
      false, // useExtendedTracking
      false, // enableCameraMonitoring
      VuforiaLocalizer.Parameters.CameraMonitorFeedback.NONE, // cameraMonitorFeedback
      0, // dx
      0, // dy
      0, // dz
      AxesOrder.XZY, // axesOrder
      90, // firstAngle
      90, // secondAngle
      0, // thirdAngle
      true
    ); // useCompetitionFieldTargetLocations
    // Initialize TFOD before waitForStart.
    // Use the Manage page to upload your custom model.
    // In the next block, replace
    // YourCustomModel.tflite with the name of your
    // custom model.
    mBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    mFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    Red = "Red";
    Blue = "Blue";
    Green = "Green";
    isRedDetected = false;
    isBlueDetected = false;
    isGreenDetected = false;
    // Set isModelTensorFlow2 to true if you used a TensorFlow
    // 2 tool, such as ftc-ml, to create the model.
    //
    // Set isModelQuantized to true if the model is
    // quantized. Models created with ftc-ml are quantized.
    //
    // Set inputSize to the image size corresponding to the model.
    // If your model is based on SSD MobileNet v2
    // 320x320, the image size is 300 (srsly!).
    // If your model is based on SSD MobileNet V2 FPNLite 320x320, the image size is 320.
    // If your model is based on SSD MobileNet V1 FPN 640x640 or
    // SSD MobileNet V2 FPNLite 640x640, the image size is 640.
    tfod.useModelFromFile(
      "model_20221121_182459.tflite",
      JavaUtil.createListWith(Blue, Green, Red),
      true,
      true,
      300
    );
    tfod.initialize(vuforiaPOWERPLAY, (float) 0.7, true, true);
    tfod.setClippingMargins(0, 80, 0, 0);
    tfod.activate();
    // Enable following block to zoom in on target.
    tfod.setZoom(1.5, 4 / 3);
    telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
    telemetry.addData(">", "Press Play to start");
    telemetry.update();
    // Wait for start command from Driver Station.
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        Lights.setPower(1);
        // Get a list of recognitions from TFOD.
        recognitions = tfod.getRecognitions();
        // If list is empty, inform the user. Otherwise, go
        // through list and display info for each recognition.
        if (JavaUtil.listLength(recognitions) == 0) {
          telemetry.addData("TFOD", "No items detected.");
        } else {
          index = 0;
          // Iterate through list and call a function to
          // display info for each recognized object.
          for (Recognition recognition_item : recognitions) {
            recognition = recognition_item;
            // Display info.
            displayInfo(index);
            // Increment index.
            index = index + 1;
          }
        }
        telemetry.update();
        if (isGreenDetected) {
          sGrabOne.setPower(1);
        }
        if (isBlueDetected) {
          sGrabTwo.setPower(1);
        }
        if (isRedDetected) {
          Lights.setPower(0);
        }
      }
    }
    // Deactivate TFOD.
    tfod.deactivate();

    androidTextToSpeech.close();
    vuforiaPOWERPLAY.close();
    tfod.close();
  }
}
