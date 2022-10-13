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

import java.util.concurrent.atomic.AtomicBoolean;

@Autonomous
public class TestingNormalization extends LinearOpMode {
    MecDrive drive;
    ScoringSystem2 score;
    Constants constants;
    Thread armThread, feedForward;
    AtomicBoolean hold, armUp, armDown;

    BNO055IMU imu;
    ColorRangeSensor distance, color;




    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecDrive(hardwareMap, false, telemetry);
        constants = new Constants();
        score = new ScoringSystem2(hardwareMap, constants);
        hold = new AtomicBoolean(false);
        armUp = new AtomicBoolean(false);
        armDown = new AtomicBoolean(false);

        score.setLinkagePosition(0.03);
        score.setGrabberPosition(constants.grabbing);

        distance = hardwareMap.get(ColorRangeSensor.class, "distance");
        color = hardwareMap.get(ColorRangeSensor.class, "color");

        color.setGain(600);
        distance.setGain(300);

        armThread = new Thread(){
            @Override
            public void run() {
                //score.setLinkagePosition(0.7);
                while(opModeIsActive()) {
                    if(armUp.get()) {
                        hold.set(false);
                        score.moveToPosition(830, 1);
                        score.setLinkagePosition(1);
                        armUp.set(false);
                        hold.set(true);
                    }else if(armDown.get()){
                        hold.set(false);
                        score.setLinkagePosition(0.7);
                        score.moveToPosition(0, 0.5);
                        score.setLinkagePosition(0.05);
                        armDown.set(false);
                    }
                }

                //Might need this
                //hold.set(true);
            }
        };



        feedForward = new Thread(){
            @Override
            public void run() {
                while(opModeIsActive()){
                    if(hold.get()){
                        score.setPower(0.2);
                    }
                }
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

        armThread.start();
        feedForward.start();



        drive.simpleMoveToPosition(-1600, MecDrive.MovementType.STRAIGHT, 0.3);
        tankRotate(Math.PI / 4, 0.3);

        armUp.set(true);

        drive.simpleMoveToPosition(60, MecDrive.MovementType.STRAIGHT, 0.2);
        sleep(1000);
        score.setGrabberPosition(0.7);
        sleep(1000);
        score.setGrabberPosition(constants.grabbing);

        armDown.set(true);

        //drive.simpleMoveToPosition(140, MecDrive.MovementType.STRAIGHT, 0.3);

        tankRotate(Math.PI / 2, 0.3);

        score.setGrabberPosition(0.7);






        //Dont know if need to check multiple time
        while(color.getNormalizedColors().red < 0.42 && color.getNormalizedColors().blue < 0.65){

            drive.setPowerAuto(0.2, MecDrive.MovementType.STRAIGHT);
            telemetry.addData("blue", color.getNormalizedColors().blue);
            telemetry.addData("red", color.getNormalizedColors().red);
            telemetry.update();
        }

        drive.simpleBrake();



        drive.simpleMoveToPosition(-60, MecDrive.MovementType.STRAFE, 0.3);

        score.setGrabberPosition(0.7);

        for(int i = 0; i < 3; i++) {

            if(i != 0){
                while(color.getNormalizedColors().red < 0.42 && color.getNormalizedColors().blue < 0.65){

                    drive.setPowerAuto(0.4, MecDrive.MovementType.RDIAGONAL);
                    telemetry.addData("blue", color.getNormalizedColors().blue);
                    telemetry.addData("red", color.getNormalizedColors().red);
                    telemetry.update();
                }

                drive.simpleBrake();



                drive.simpleMoveToPosition(-40, MecDrive.MovementType.STRAFE, 0.3);


            }

            score.setLinkagePosition(0.2 - (i * 0.03));


            while (distance.getDistance(DistanceUnit.CM) > 4.3) {
                drive.setPowerAuto(0.2, MecDrive.MovementType.STRAIGHT);

                telemetry.addData("distance", distance.getDistance(DistanceUnit.CM));
                telemetry.update();

            }
            drive.simpleBrake();


            score.setGrabberPosition(constants.grabbing);
            sleep(600);

            score.moveToPosition(200, 1);
            hold.set(true);

            drive.simpleMoveToPosition(-650, MecDrive.MovementType.STRAIGHT, 0.3);
            score.setLinkagePosition(0.7);

            tankRotate(Math.PI / 4.35, 0.3);

            armUp.set(true);

            drive.simpleMoveToPosition(-50, MecDrive.MovementType.STRAIGHT, 0.3);

            sleep(1000);
            score.setGrabberPosition(0.7);
            sleep(1000);

            armDown.set(true);

            //drive.simpleMoveToPosition(70, MecDrive.MovementType.STRAIGHT, 0.4);

            tankRotate(Math.PI / 2, 0.3);
            sleep(500);
        }

        //Will have to check if this aligns straight already (need color sensor or not) ->
        // may need to turn into slight diagonal instead of straight to check color
        //drive.simpleMoveToPosition(675, MecDrive.MovementType.STRAIGHT, 0.3);


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
