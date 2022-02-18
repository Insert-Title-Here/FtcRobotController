package org.firstinspires.ftc.teamcode.MecanumCode.Auto.NewTank;

import android.view.animation.RotateAnimation;

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


@Autonomous(name="Testing New Tank")
public class Testing extends OpModeWrapper {

    MecanumDriveTrain drive;
    Carousel carousel;
    CapstoneArm capArm;
    MagneticArm magArm;
    // The IMU sensor object

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    WebcamName wc;
    OpenCvCamera camera;

    Thread armMovementThread;
    Thread telemetryThread;
    private volatile boolean moveArm;


    static final BarcodePipelineBlue bPipeline = new BarcodePipelineBlue();
    static BarcodePipelineBlue.BarcodePosition capstonePos;

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


        armMovementThread = new Thread(){
            @Override
            public void run(){
                while(!moveArm);
                capArm.goToPosition(0);
            }
        };

        telemetryThread = new Thread(){
            @Override
            public void run(){
                while(opModeIsActive()){
                    telemetry.addData("angle", drive.getAngle());
                    telemetry.addData("degrees", (drive.getAngle() * 180)/Math.PI);

                    telemetry.addData("Pos", Math.PI/2);

                    telemetry.update();
                }
            }
        };

        while(!opModeIsActive()){
            telemetry.addData("pos", bPipeline.getPos());
            telemetry.update();
        }

        /*while(opModeIsActive()){
            telemetry.addData("angle", drive.getAngle());
            telemetry.update();
        }*/



    }

    @Override
    protected void onStart() {
        capstonePos = bPipeline.getPos();
        armMovementThread.start();
        telemetryThread.start();
        camera.stopStreaming();

        //telemetry.addData("hi", "hi");

        /*drive.driveAuto(120, 240, MecanumDriveTrain.MovementType.STRAIGHT);
        drive.driveAuto(120, 240, MecanumDriveTrain.MovementType.STRAFE);
        drive.driveAuto(120, 240, MecanumDriveTrain.MovementType.ROTATE);

         */
        // Forward: 1 ft 540.3 tics (5403 for 10 ft)
        // Rotation: 360 degrees 3665 tics
        // Strafe: 590 tics/ft - = Left, + = Right


        //drive.driveAuto(0.3, -200, MecanumDriveTrain.MovementType.STRAIGHT);
        drive.setPowerAuto(0.3, MecanumDriveTrain.MovementType.ROTATE);
        sleep(3000);
        /*
        drive.driveAuto(0.3, 1000, MecanumDriveTrain.MovementType.ROTATE);
        drive.driveAuto(0.3, -600, MecanumDriveTrain.MovementType.STRAIGHT);
        carousel.spinCarousel(-5000, this, Carousel.CarouselMode.AUTO);

        drive.driveAuto(0.3, 500, MecanumDriveTrain.MovementType.STRAFE);
        drive.driveAuto(0.3, -200, MecanumDriveTrain.MovementType.STRAIGHT);
        drive.driveAuto(0.3, 500, MecanumDriveTrain.MovementType.STRAIGHT);
        drive.driveAuto(0.3, 1900, MecanumDriveTrain.MovementType.ROTATE);
        //drive.driveAuto(0.3, -520, MecanumDriveTrain.MovementType.STRAIGHT);
        if(capstonePos == BarcodePipelineBlue.BarcodePosition.RIGHT) {
            drive.driveAuto(0.3, -510, MecanumDriveTrain.MovementType.STRAIGHT);
            capArm.goToPosition(Constants.TOP_GOAL_POS);
            drive.driveAuto(0.3, -20, MecanumDriveTrain.MovementType.STRAIGHT);


            capArm.toggleGrab();

            sleep(1000);
            capArm.toggleGrab();


        }else if(capstonePos == BarcodePipelineBlue.BarcodePosition.CENTER){
            drive.driveAuto(0.3, -555, MecanumDriveTrain.MovementType.STRAIGHT);
            capArm.goToPosition(Constants.MID_GOAL_POS);
            drive.driveAuto(0.3, -20, MecanumDriveTrain.MovementType.STRAIGHT);

            capArm.toggleGrab();

            sleep(1000);
            capArm.toggleGrab();


        }else{
            drive.driveAuto(0.3, -600, MecanumDriveTrain.MovementType.STRAIGHT);
            capArm.goToPosition(Constants.BOTTOM_GOAL_POS);

            capArm.toggleGrab();

            sleep(1000);
            capArm.toggleGrab();
            drive.driveAuto(0.3, 50, MecanumDriveTrain.MovementType.STRAIGHT);

        }
        //capArm.goToPosition(300);
        //capArm.toggleGrab();
        moveArm = true;
        sleep(1000);

        drive.driveAuto(0.3, 1200, MecanumDriveTrain.MovementType.STRAIGHT);
        drive.driveAuto(0.3, 800, MecanumDriveTrain.MovementType.STRAFE);

        */
        //drive.driveAuto(0.5, 1832, MecanumDriveTrain.MovementType.ROTATE);
        //drive.driveAuto(0.3, 2500, MecanumDriveTrain.MovementType.STRAFE);
        //drive.driveAuto(0.3, 880, MecanumDriveTrain.MovementType.STRAIGHT);

        /*drive.driveAuto(0.3, -2500, MecanumDriveTrain.MovementType.STRAFE);
        drive.driveAuto(0.3, -1900, MecanumDriveTrain.MovementType.ROTATE);
        drive.driveAuto(0.2, -460, MecanumDriveTrain.MovementType.STRAIGHT);
        drive.driveAuto(0.1, -150, MecanumDriveTrain.MovementType.STRAIGHT);


        carousel.spinCarousel(4000, this, Carousel.CarouselMode.AUTO);
        drive.driveAuto(0.3, 720, MecanumDriveTrain.MovementType.STRAIGHT);


         */


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
