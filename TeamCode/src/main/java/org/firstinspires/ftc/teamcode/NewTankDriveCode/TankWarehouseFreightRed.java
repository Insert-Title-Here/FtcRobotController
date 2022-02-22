package org.firstinspires.ftc.teamcode.NewTankDriveCode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.MecanumCode.Auto.Vision.BarcodePipelineWarehouseRed;
import org.firstinspires.ftc.teamcode.MecanumCode.Common.CapstoneArm;
import org.firstinspires.ftc.teamcode.MecanumCode.Common.Carousel;
import org.firstinspires.ftc.teamcode.MecanumCode.Common.Constants;
import org.firstinspires.ftc.teamcode.MecanumCode.Common.MagneticArm;
import org.firstinspires.ftc.teamcode.MecanumCode.Common.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.MecanumCode.Common.OpModeWrapper;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.io.FileNotFoundException;


@Autonomous(name="WarehouseFreight Red Tank")
public class TankWarehouseFreightRed extends OpModeWrapper {

    MecanumDriveTrain drive;
    Carousel carousel;
    CapstoneArm capArm;
    MagneticArm magArm;
    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    WebcamName wc;
    OpenCvCamera camera;

    //static final BarcodePipeline.AutoSide side = BarcodePipeline.AutoSide.RED;

    Thread armMovementThread;
    private volatile boolean moveArm;


    // global obj
    BarcodePipelineWarehouseRed.BarcodePosition capstonePos;
    static final BarcodePipelineWarehouseRed bPipeline = new BarcodePipelineWarehouseRed();

    @Override
    protected void onInitialize() throws FileNotFoundException {
        drive = new MecanumDriveTrain(hardwareMap);
        carousel = new Carousel(hardwareMap);
        capArm = new CapstoneArm(hardwareMap);
        magArm = new MagneticArm(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        wc = hardwareMap.get(WebcamName.class, "Webcam");

        // W/ or W/ out live preview
        camera = OpenCvCameraFactory.getInstance().createWebcam(wc, cameraMonitorViewId);
        // camera = OpenCvCameraFactory.getInstance().createWebcam(wc);

        camera.setPipeline(bPipeline);

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


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        moveArm = false;
        armMovementThread = new Thread(){
            @Override
            public void run(){
                while(!moveArm);
                capArm.goToPosition(0);



            }
        };




        while(!opModeIsActive()){
            telemetry.addData("pos", bPipeline.getPos());
            telemetry.update();
        }

        capArm.setGrabberPosition(0.8);


    }

    @Override
    protected void onStart() {
        armMovementThread.start();
        capstonePos = bPipeline.getPos();
        drive.tankRotate(-Math.PI/2, 0.3);
        drive.driveAuto(0.3,1200, MecanumDriveTrain.MovementType.STRAIGHT);
        drive.tankRotate(0, 0.3);


        if (capstonePos == BarcodePipelineWarehouseRed.BarcodePosition.RIGHT) {
            //drive.driveAuto(0.3, -520, MecanumDriveTrain.MovementType.STRAIGHT);
            //capArm.goToPosition(300);
            //drive.driveAuto(0.3, -160, MecanumDriveTrain.MovementType.STRAIGHT);
            capArm.goToPosition(Constants.TOP_GOAL_POS);
            drive.driveAuto(0.3, -650, MecanumDriveTrain.MovementType.STRAIGHT);

            capArm.toggleGrab();

            sleep(500);
            capArm.toggleGrab();

            drive.driveAuto(0.3, 650, MecanumDriveTrain.MovementType.STRAIGHT);

        } else if (capstonePos == BarcodePipelineWarehouseRed.BarcodePosition.CENTER) {
            //drive.driveAuto(0.3, -180, MecanumDriveTrain.MovementType.STRAIGHT);
            capArm.goToPosition(Constants.MID_GOAL_POS);
            drive.driveAuto(0.3, -700, MecanumDriveTrain.MovementType.STRAIGHT);

            capArm.toggleGrab();
            sleep(500);
            capArm.toggleGrab();

            drive.driveAuto(0.3, 700, MecanumDriveTrain.MovementType.STRAIGHT);



        } else {
            drive.driveAuto(0.3, -750, MecanumDriveTrain.MovementType.STRAIGHT);
            capArm.goToPosition(Constants.BOTTOM_GOAL_POS);
            capArm.toggleGrab();
            sleep(500);
            capArm.toggleGrab();

            drive.driveAuto(0.3, 750, MecanumDriveTrain.MovementType.STRAIGHT);
            //drive.driveAuto(0.3, -100, MecanumDriveTrain.MovementType.STRAFE);


            //drive.driveAuto(0.3, 40, MecanumDriveTrain.MovementType.STRAIGHT);
        }



        moveArm = true;

        drive.driveAuto(0.3, 300, MecanumDriveTrain.MovementType.STRAIGHT);

        drive.tankRotate(-15*Math.PI/24, 0.3);

        drive.driveAuto(0.3, -2400, MecanumDriveTrain.MovementType.STRAIGHT);
        drive.tankRotate(0, 0.5);
        drive.driveAuto(0.3, -1000, MecanumDriveTrain.MovementType.STRAIGHT);
        drive.tankRotate(Math.PI/4, 0.3);

    }
    @Override
    protected void onStop() {

    }

}
