package org.firstinspires.ftc.teamcode.League1.Testing.AutonomousTesting;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.League1.Common.Constants;
import org.firstinspires.ftc.teamcode.League1.Subsystems.MecDrive;
import org.firstinspires.ftc.teamcode.League1.Subsystems.ScoringSystem2;

@Autonomous
public class TestingNormalization extends LinearOpMode {
    MecDrive drive;
    ScoringSystem2 score;
    Constants constants;
    Thread armUpThread, armDownThread;

    BNO055IMU imu;
    ColorRangeSensor distance, color;




    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecDrive(hardwareMap, false, telemetry);
        constants = new Constants();
        score = new ScoringSystem2(hardwareMap, constants);

        score.setLinkagePosition(0.03);
        score.setGrabberPosition(constants.grabbing);

        distance = hardwareMap.get(ColorRangeSensor.class, "distance");
        color = hardwareMap.get(ColorRangeSensor.class, "color");

        color.setGain(600);
        distance.setGain(300);

        armUpThread = new Thread(){
            @Override
            public void run() {
                //score.setLinkagePosition(0.7);
                score.moveToPosition(830, 0.8);
                score.setLinkagePosition(0.95);
            }
        };

        armDownThread = new Thread(){
            @Override
            public void run() {
                score.setLinkagePosition(0.7);
                score.moveToPosition(0, 0.5);
                score.setLinkagePosition(0.95);

            }
        };


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);



        waitForStart();



        drive.simpleMoveToPosition(-1600, MecDrive.MovementType.STRAIGHT, 0.3);
        tankRotate(Math.PI / 2, 0.4);

        while(color.getNormalizedColors().red < 0.52 && color.getNormalizedColors().blue < 0.65){

            drive.setPowerAuto(0.15, MecDrive.MovementType.STRAIGHT);
            telemetry.addData("blue", color.getNormalizedColors().blue);
            telemetry.addData("red", color.getNormalizedColors().red);
            telemetry.update();
        }

        drive.simpleBrake();



        drive.simpleMoveToPosition(-50, MecDrive.MovementType.STRAFE, 0.3);
        score.setLinkagePosition(0.2);

        score.setGrabberPosition(constants.openAuto);

        while(distance.getDistance(DistanceUnit.CM) > 6.2){
            drive.setPowerAuto(0.15, MecDrive.MovementType.STRAIGHT);

            telemetry.addData("distance", distance.getDistance(DistanceUnit.CM));
            telemetry.update();

        }

        drive.simpleBrake();

        score.setGrabberPosition(constants.grabbing);
        sleep(300);

        score.setLinkagePosition(0.7);

        drive.simpleMoveToPosition(-675, MecDrive.MovementType.STRAIGHT, 0.3);

        tankRotate(Math.PI / 5.2, 0.3);

        armUpThread.start();

        score.setGrabberPosition(constants.openAuto);



    }


    public void tankRotate(double radians, double power){
        if(radians > imu.getAngularOrientation().firstAngle){
            power *= -1;
        }

        while(Math.abs(imu.getAngularOrientation().firstAngle - radians) > (Math.PI/32)){
            drive.setPowerAuto(power, MecDrive.MovementType.ROTATE);
        }

        drive.simpleBrake();




    }
}
