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


@Autonomous(name="TestingHeading")
public class TestingHeading extends OpModeWrapper {

    MecanumDriveTrain drive;

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    WebcamName wc;
    OpenCvCamera camera;





    @Override
    protected void onInitialize() throws FileNotFoundException {
        drive = new MecanumDriveTrain(hardwareMap);


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());



            // Method will be called if the camera cannot be opened


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






    }


    //Straight is about 1 ft to 508 tics
    //Strafe is about 1 ft to 640 tics

    @Override
    protected void onStart() {
        /*
        drive.driveAuto(0.3, 508, MecanumDriveTrain.MovementType.STRAIGHT);
        sleep(1000);
        drive.driveAuto(0.3, 640, MecanumDriveTrain.MovementType.STRAFE);

         */

        //drive.straightHeadingTester(Math.PI, 1);

        //sleep(1000);

        /*

        drive.straightHeadingTester((3 * Math.PI / 4), (int)(2 * Math.sqrt(2)));
        sleep(1000);
        drive.straightHeadingTester(Math.PI / 4, (int)(2 * Math.sqrt(2)));
        sleep(1000);
        drive.straightHeadingTester(5 * Math.PI / 4,(int)(2 * Math.sqrt(2)));
        sleep(1000);
        drive.straightHeadingTester(7 * Math.PI / 4,(int)(2 * Math.sqrt(2)));

         */

        drive.straightHeadingTester(-Math.PI / 4, (int)(2 * Math.sqrt(2)));
        drive.straightHeadingTester(-Math.PI / 2, (int)(2 * Math.sqrt(2)));






    }

    @Override
    protected void onStop() {
        drive.brake();
    }



}
