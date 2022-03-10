package org.firstinspires.ftc.teamcode.MecanumCode.Auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.MecanumCode.Auto.Vision.BarcodePipelineBlue;
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


@Autonomous(name="Warehouse Freight Blue Confirmation")
public class WarehouseFreightBlue extends OpModeWrapper {

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

    Thread armMovementThread;
    private volatile boolean moveArm;


    BarcodePipelineBlue.BarcodePosition capstonePos;

    static final BarcodePipelineBlue brp = new BarcodePipelineBlue();







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

        moveArm = false;
        armMovementThread = new Thread(){
            @Override
            public void run(){
                while(!moveArm);
                capArm.goToPosition(0);
            }
        };

        //magArm.setMagnetPosition(MagneticArm.magnetState.OPEN);


        while(!opModeIsActive()){
            telemetry.addData("pos", brp.getPos());
            telemetry.update();
        }

        capArm.setGrabberPosition(0.8);


    }

    @Override
    protected void onStart() {
        armMovementThread.start();

        capstonePos = brp.getPos();

        /*drive.driveAuto(120, 240, MecanumDriveTrain.MovementType.STRAIGHT);
        drive.driveAuto(120, 240, MecanumDriveTrain.MovementType.STRAFE);
        drive.driveAuto(120, 240, MecanumDriveTrain.MovementType.ROTATE);

         */
        // Forward: 1 ft 540.3 tics (5403 for 10 ft)
        // Rotation: 360 degrees 3665 tics
        // Strafe: 590 tics/ft - = Left, + = Right


        drive.driveAuto(0.3, -450, MecanumDriveTrain.MovementType.RDIAGONAL);
        sleep(2000);
        drive.driveAuto(0.3, -1250, MecanumDriveTrain.MovementType.STRAIGHT);
        drive.driveAuto(0.4, 1000, MecanumDriveTrain.MovementType.ROTATE);

        /*
        drive.driveAuto(0.3, -200, MecanumDriveTrain.MovementType.STRAIGHT);
        drive.driveAuto(0.3, -1650, MecanumDriveTrain.MovementType.STRAFE);
        drive.driveAuto(0.3, -520, MecanumDriveTrain.MovementType.STRAIGHT);


         */
        //capArm.goToPosition(300);

        //To lock path:
        // capstonePos = BarcodePipelineBlue.BarcodePosition.LEFT;

        if(capstonePos == BarcodePipelineBlue.BarcodePosition.RIGHT) {
            //drive.driveAuto(0.3, -520, MecanumDriveTrain.MovementType.STRAIGHT);
            capArm.goToPosition(Constants.TOP_GOAL_POS);
            capArm.toggleGrab();
            sleep(2000);
            capArm.toggleGrab();
        }else if(capstonePos == BarcodePipelineBlue.BarcodePosition.LEFT){
            //drive.driveAuto(0.3, -480, MecanumDriveTrain.MovementType.STRAIGHT);
            capArm.goToPosition(Constants.BOTTOM_GOAL_POS);
            drive.driveAuto(0.3, -100, MecanumDriveTrain.MovementType.STRAIGHT);
            capArm.toggleGrab();
            sleep(1000);
            capArm.toggleGrab();
            drive.driveAuto(0.3, 100, MecanumDriveTrain.MovementType.STRAIGHT);
        }else{
            //drive.driveAuto(0.3, -460, MecanumDriveTrain.MovementType.STRAIGHT);
            capArm.goToPosition(Constants.MID_GOAL_POS);
            //drive.driveAuto(0.3, -200, MecanumDriveTrain.MovementType.ROTATE);
            //drive.driveAuto(0.3, -50, MecanumDriveTrain.MovementType.STRAIGHT);
            capArm.toggleGrab();
            sleep(1000);
            capArm.toggleGrab();
            //drive.driveAuto(0.3, 200, MecanumDriveTrain.MovementType.STRAIGHT);
            //drive.driveAuto(0.3, 70, MecanumDriveTrain.MovementType.ROTATE);
        }

        moveArm = true;

        // Instead of -2200 strafe:

        drive.driveAuto(0.3, -1000, MecanumDriveTrain.MovementType.ROTATE);
        drive.driveAuto(0.3, 1500, MecanumDriveTrain.MovementType.STRAIGHT);
        drive.driveAuto(0.3, 1070, MecanumDriveTrain.MovementType.ROTATE);
        drive.driveAuto(0.3, -340, MecanumDriveTrain.MovementType.STRAFE);

        sleep(1000);

        //drive.driveAuto(0.3, 200, MecanumDriveTrain.MovementType.ROTATE);
        drive.driveAuto(0.3, 1700, MecanumDriveTrain.MovementType.STRAIGHT);
        drive.driveAuto(0.3, 1300, MecanumDriveTrain.MovementType.STRAFE);

        //Freight Pickup
        magArm.setMagnetPosition(MagneticArm.magnetState.GRABBING);
        drive.tankRotate(-Math.PI/4, 0.3);
        magArm.setArmPosition(Constants.MAGARM_FREIGHT);
        drive.driveAuto(0.3, 600, MecanumDriveTrain.MovementType.STRAIGHT);
        magArm.setLevelPosition(Constants.LEVEL_DOWN_POS);
        sleep(2000);
        for (int i = 0; i < 2; i++){
            drive.tankRotate(-Math.PI/6, 0.3);
            sleep(500);
            drive.tankRotate(-Math.PI/3, 0.3);
        }
        sleep(1000);
        magArm.setLevelPosition(Constants.LEVEL_HALF_POS);
        sleep(1000);
        drive.driveAuto(0.3, -500, MecanumDriveTrain.MovementType.STRAIGHT);
        magArm.setLevelPosition(Constants.LEVEL_DOWN_POS);
        sleep(1000);


        //drive.driveAuto(0.3, 2500, MecanumDriveTrain.MovementType.STRAIGHT);


        //double angle = imu.getAngularOrientation().firstAngle;
        //drive.writeLoggerToFile();
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
