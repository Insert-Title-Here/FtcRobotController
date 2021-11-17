 package org.firstinspires.ftc.teamcode;

 import com.qualcomm.hardware.bosch.BNO055IMU;
 import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
 import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.hardware.DcMotor;
 import com.qualcomm.robotcore.hardware.Servo;

 import org.firstinspires.ftc.robotcore.external.Func;
 import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
 import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
 import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
 import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
 import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
 import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
 import org.openftc.easyopencv.OpenCvCamera;
 import org.openftc.easyopencv.OpenCvCameraFactory;
 import org.openftc.easyopencv.OpenCvCameraRotation;

 import java.util.Locale;

 @Autonomous(name = "Freight Auto (red2)", group = "Linear Opmode")

 public class FreightScoreAutoRed2 extends LinearOpMode {
     // The IMU sensor object
     BNO055IMU imu;
     DcMotor extender;
     Servo grabber;


     double servoPosition = 0.5;


     boolean isExtended = false;
     boolean isGrabbing = true;
     boolean servoMoving = false;
     boolean previousYState;
     Thread armThread;



     // State used for updating telemetry
     Orientation angles;
     Acceleration gravity;

     WebcamName wc;
     OpenCvCamera camera;

     BarcodePipeline.BarcodePosition capstonePos;


     // global obj
     static final BarcodePipeline brp = new BarcodePipeline();

     //----------------------------------------------------------------------------------------------
     // Main logic
     //----------------------------------------------------------------------------------------------

     @Override
     public void runOpMode() {
         DriveTrain drive = new DriveTrain(hardwareMap);
         extender = hardwareMap.get(DcMotor.class, "ExtensionArm");
         extender.setDirection(DcMotor.Direction.FORWARD);
         extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         extender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
         grabber = hardwareMap.get(Servo.class, "Grabber");

         armThread = new Thread() {
             @Override
             public void run() {
                 extendArm(500);
             }
         };

         int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
         wc = hardwareMap.get(WebcamName.class, "Webcam");

         // W/ or W/ out live preview
         camera = OpenCvCameraFactory.getInstance().createWebcam(wc, cameraMonitorViewId);
         // camera = OpenCvCameraFactory.getInstance().createWebcam(wc);

         camera.setPipeline(brp);

         // Open an asynchronous connection to the device
         camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

             // Start opening the camera and stream it
             @Override
             public void onOpened() {

                 /*
                 // create a rgb2gray mat pipeline
                 class GrayPipeline extends OpenCvPipeline {
                     Mat gray = new Mat();
                     @Override
                     public Mat processFrame(Mat input) {
                         // mat src, mat dst, int code, convert rgb img to gray
                         Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY);
                         return gray;
                     }
                 } */

                 camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
             }

             // Method will be called if the camera cannot be opened
             @Override
             public void onError(int errorCode) {
                 telemetry.addData("Camera Init Error", errorCode);
             }
         });


         // Set up the parameters with which we will use our IMU. Note that integration
         // algorithm here just reports accelerations to the logcat log; it doesn't actually
         // provide positional information.
         BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
         parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
         parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
         parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
         parameters.loggingEnabled = true;
         parameters.loggingTag = "IMU";
         parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

         // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
         // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
         // and named "imu".
         imu = hardwareMap.get(BNO055IMU.class, "imu");
         imu.initialize(parameters);

         // Set up our telemetry dashboard
         //composeTelemetry();
         grabber.setPosition(0);

         // Wait until we're told to go
         waitForStart();
         capstonePos = brp.getPos();


         if (capstonePos == BarcodePipeline.BarcodePosition.CENTER) {
             //Middle Goal
             extendArm(3450);
             drive.goToPosition(-616, false, 0.3);
             grabber.setPosition(0.5);
             drive.goToPosition(316, false, 0.3);
             rotateToPosition(drive, -50);
         } else if (capstonePos == BarcodePipeline.BarcodePosition.RIGHT) {
             //Top Goal
             extendArm(6295);
             drive.goToPosition(-450, false, 0.3);
             grabber.setPosition(0.5);
             drive.goToPosition(150, false, 0.3);
         } else if (capstonePos == BarcodePipeline.BarcodePosition.LEFT) {
             //Top Goal
             extendArm(6295);
             drive.goToPosition(-450, false, 0.3);
             grabber.setPosition(0.5);
             drive.goToPosition(150, false, 0.3);
             //Bottom Goal
             /*
             extendArm(1370);
             drive.goToPosition(-740, false, 0.3);
             extendArm(1300);
             grabber.setPosition(0.3);
             drive.goToPosition(-40, false, 0.3);
             sleep(1000);
             drive.goToPosition(200, false, 0.3);

              */
         }


         extendArm(1000);
         rotateToPosition(drive, -325);
         //armThread.start();
         //sleep(2000);



         /*drive.lf.setPower(0.4);
         drive.rf.setPower(-0.4);
         sleep(1000);
         drive.lf.setPower(0);
         drive.rf.setPower(0);

          */

         drive.goToPosition(-2400, false, 1);
         extendArm(0);
         sleep(4000);

     }

     public void extendArm(int armPosition) {

         extender.setTargetPosition(armPosition);

         extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);

         extender.setPower(0.5);

         while (extender.isBusy()) {

         }

         extender.setPower(0);

         extender.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
     }


     public synchronized void grab(double position){
         double targetPosition;
         servoMoving = true;

         previousYState = gamepad1.y;

         if(isGrabbing) {
             targetPosition = 0;
         }else{
             targetPosition = position;
         }

         servoPosition = targetPosition;
         grabber.setPosition(servoPosition);

         while(previousYState == gamepad1.y){

         }
         isGrabbing = !isGrabbing;

     }

     public void rotateToPosition(DriveTrain drive, int tics) {
         drive.lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         drive.lf.setTargetPosition(tics);
         drive.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);

         drive.rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         drive.rf.setTargetPosition(-tics);
         drive.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);

         drive.lf.setPower(0.2);
         drive.rf.setPower(0.2);

         while (drive.lf.isBusy() && drive.rf.isBusy()) {

         }

         drive.brake();
     }


 }
