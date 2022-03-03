package org.firstinspires.ftc.teamcode.MecanumCode.Auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.MecanumCode.Auto.Vision.BarcodePipelineBlue;
import org.firstinspires.ftc.teamcode.MecanumCode.Common.CapstoneArm;
import org.firstinspires.ftc.teamcode.MecanumCode.Common.Carousel;
import org.firstinspires.ftc.teamcode.MecanumCode.Common.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.MecanumCode.Common.OpModeWrapper;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.io.FileNotFoundException;

@Disabled
@Autonomous(name="Carousel Park Red")
public class CarouselParkRed extends OpModeWrapper {

    MecanumDriveTrain drive;
    Carousel carousel;
    CapstoneArm capArm;
    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    WebcamName wc;
    OpenCvCamera camera;

    BarcodePipelineBlue.BarcodePosition capstonePos;

    static final BarcodePipelineBlue brp = new BarcodePipelineBlue();







    @Override
    protected void onInitialize() throws FileNotFoundException {
        drive = new MecanumDriveTrain(hardwareMap);
        carousel = new Carousel(hardwareMap);
        capArm = new CapstoneArm(hardwareMap);

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

        capArm.setGrabberPosition(0.8);


    }

    @Override
    protected void onStart() {

        /*drive.driveAuto(120, 240, MecanumDriveTrain.MovementType.STRAIGHT);
        drive.driveAuto(120, 240, MecanumDriveTrain.MovementType.STRAFE);
        drive.driveAuto(120, 240, MecanumDriveTrain.MovementType.ROTATE);

         */
        // Forward: 1 ft 540.3 tics (5403 for 10 ft) + = forwards, - = backwards
        // Rotation: 360 degrees 3665 tics
        // Strafe: 590 tics/ft - = Left, + = Right
        drive.driveAuto(0.3, -200, MecanumDriveTrain.MovementType.STRAIGHT);
        drive.driveAuto(0.3, 1650, MecanumDriveTrain.MovementType.STRAFE);
        if(capstonePos == BarcodePipelineBlue.BarcodePosition.RIGHT) {
            drive.driveAuto(0.3, -520, MecanumDriveTrain.MovementType.STRAIGHT);
            capArm.goToPosition(300);
        }else if(capstonePos == BarcodePipelineBlue.BarcodePosition.LEFT){
            drive.driveAuto(0.3, -480, MecanumDriveTrain.MovementType.STRAIGHT);
            capArm.goToPosition(1300);

        }else{
            drive.driveAuto(0.3, -460, MecanumDriveTrain.MovementType.STRAIGHT);
            capArm.goToPosition(2325);
        }
        //drive.driveAuto(0.3, -540, MecanumDriveTrain.MovementType.STRAIGHT);
        capArm.toggleGrab();

        drive.driveAuto(0.3, 540, MecanumDriveTrain.MovementType.STRAIGHT);
        drive.driveAuto(0.3, 900, MecanumDriveTrain.MovementType.ROTATE);
        drive.driveAuto(0.3, 2000, MecanumDriveTrain.MovementType.STRAIGHT);


        //double angle = imu.getAngularOrientation().firstAngle;
        drive.writeLoggerToFile();
/*
        while(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).firstAngle < Math.PI){
            drive.setPowerAuto(0.3, MecanumDriveTrain.MovementType.ROTATE);

            telemetry.addData("Radians: ", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.RADIANS).firstAngle);
            telemetry.update();

        }

 */


        //moveTest(3000);
        /*
        drive.setPower(0.5, 0, 0, -0.5);
        sleep(1000);
        drive.setPower(-0.5, 0, 0, 0.5);
        sleep(1000);
        drive.setPower(0,0,0,0);


         */
    }

    @Override
    protected void onStop() {

    }


    /*private void moveTest(int motorTics){


        for(DcMotor motor: drive.getMotors()){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setTargetPosition(motorTics);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }



        drive.setPowerAuto(0.5, MecanumDriveTrain.MovementType.STRAIGHT);

        while(drive.fr.getCurrentPosition() < motorTics){
            telemetry.addData("FL Tics", drive.fl.getCurrentPosition());
            telemetry.addData("FR Tics", drive.fr.getCurrentPosition());
            telemetry.addData("BL Tics", drive.bl.getCurrentPosition());
            telemetry.addData("BR Tics", drive.br.getCurrentPosition());
            telemetry.update();

        }


        drive.setPowerAuto(0, MecanumDriveTrain.MovementType.STRAIGHT);


    }*/
}
