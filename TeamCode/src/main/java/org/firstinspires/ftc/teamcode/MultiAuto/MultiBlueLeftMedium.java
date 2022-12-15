package org.firstinspires.ftc.teamcode.MultiAuto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auto.ContourMultiScoreLeft;
import org.firstinspires.ftc.teamcode.Common.Constants;
import org.firstinspires.ftc.teamcode.Common.MecanumDrive;
import org.firstinspires.ftc.teamcode.Common.ScoringSystem;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.atomic.AtomicBoolean;

@Autonomous @Config
public class MultiBlueLeftMedium extends LinearOpMode {
    // instantiating
    Thread liftThread;
    MecanumDrive drive;
    ContourMultiScoreLeft detect1;
    ScoringSystem score;
    Constants constants;
    OpenCvWebcam webcam;
    AtomicBoolean cont;

    // ftc dashboard values + properCX
    private double properCX = 187; //67
    private double properCXLow = 171;
    public static int positive_negative = 1;
    public static int turnDenom = 4;
    public static boolean toggle = false;




    @Override
    public void runOpMode() throws InterruptedException {
        // value initializing
        detect1 = new ContourMultiScoreLeft(telemetry);
        drive = new MecanumDrive(hardwareMap, telemetry);
        score = new ScoringSystem(hardwareMap, telemetry);
        constants = new Constants();
        cont = new AtomicBoolean();
        cont.set(false);

        // camera setup
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(detect1);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                webcam.startStreaming(320, 176, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Init Error", errorCode);
                telemetry.update();
            }
        });

        // thread for keeping lift up
        liftThread = new Thread() {
            @Override
            public void run(){
                while(opModeIsActive()){
                    if((score.getEncoderPosition() > 100 && cont.get())){
                        score.setPower(constants.getSteadyPow());
                    }

//
                 telemetry.addData("imu", drive.currentAngle());
                   telemetry.update();
                }

            }
        };

        // code to turn servo of cam
        //score.setCamPosition(constants.getSleeveCamPos());
        //detect1.park = true;

        // ftc dashboard setup
        FtcDashboard.getInstance().startCameraStream(webcam, 0);
        telemetry.addData("Status", "Initialized");

        telemetry.update();

        //close claw
        score.setClawPosition(constants.getClawClosePos()); //TODO

        // turn servo of cam forward for poles
        score.setCamPosition(constants.getStrafeMedCamPos());


        waitForStart();
        liftThread.start();
        //detect1.park = false;



        if (toggle) {

            //lift claw a little bit
            score.goToPosition(50, 0.7);
            sleep(200);
            // go forward next to pole
            drive.goToPositionPID(1060, "go forward next to pole");
            // turn to left 45 degrees to medium pole
            drive.turn45(-Math.PI / 4);
            // go to pole a bit
            drive.goToPosition(0.3, 0.3, 0.3, 0.3, 90, "go forward some to pole");
            sleep(100);

            boolean right = true;
            boolean left = true;

            // camera position correction
            while (detect1.getcX() < properCX - 5 || detect1.getcX() > properCX + 5) {
                telemetry.addData("success", "sucess");
                telemetry.update();
                if (detect1.getcX() < properCX - 5 && right) {
                    // strafe to the right
                    //                drive.goToPosition(0.15, -0.15, -0.15, 0.15);
                    drive.goToPosition(-0.2, 0.2, 0.2, -0.2);

                    telemetry.addData("success 1", "sucess");
                    telemetry.update();
                    right = false;
                }
                if (detect1.getcX() > properCX + 5 && left) {
                    // strafe to the left (change fr and bl)
                    drive.goToPosition(0.2, -0.2, -0.2, 0.2);

                    //                drive.goToPosition(-0.15, 0.15, 0.15, -0.15);

                    telemetry.addData("success 2", "sucess");
                    telemetry.update();
                    left = false;
                }
                //            if (detect1.getcX() >= properCX - 5 && detect1.getcX() <= properCX + 5) {
                //                drive.goToPosition(0, 0, 0, 0);
                //            }
            }

            drive.goToPosition(0, 0, 0, 0);

            sleep(200);

            scoreConeMed(438, 416, 437, 426);

            // turn back straight
            drive.turn45(Math.PI / 4);
            //moves robot to correct parking position
            //        if (detect1.getParkPosition() == ContourMultiScore.ParkingPosition.LEFT) {
            //            // move to left park (strafe right)
            //            drive.goToPosition(-0.3, -0.3, -0.3, -0.3, 1372, "move backwards");
            //            drive.goToPosition(-0.3, 0.3, 0.3, -0.3 , 200, "strafe right");
            //
            //
            //
            //        } else if (detect1.getParkPosition() == ContourMultiScore.ParkingPosition.CENTER) {
            //            // move to center park (don't move at all)
            //            drive.goToPosition(-0.3, 0.3, 0.3, -0.3 , 200, "strafe right");
            //
            //        } else {
            //            // move to right park (strafe more left)
            //            drive.goToPosition(0.3, 0.3, 0.3, 0.3 , 1000, "move forward");
            //            drive.goToPosition(-0.3, 0.3, 0.3, -0.3 , 200, "strafe right");
            //
            //
            //        }


            score.setClawPosition(constants.getClawOpenPos());


            //go forward to blue cone tape adjacent mat
            drive.goToPosition(0.4, 0.4, 0.4, 0.4, drive.avgPosition(828, 856, 941, 926), "go forward to next mat");



    /*

            // back up
            drive.goToPosition(-0.4, -0.4, -0.4, -0.4, -284, "back up");


            // turn 90 to the left
            drive.turn(Math.PI / 2);
            right = true;
            left = true;

            // camera position correction
            while (detect1.getcX() < properCX - 5 || detect1.getcX() > properCX + 5) {
                telemetry.addData("success", "sucess");
                telemetry.update();
                if (detect1.getcX() < properCX - 5 && right) {
                    // strafe to the right
    //                drive.goToPosition(0.15, -0.15, -0.15, 0.15);
                    drive.goToPosition(-0.2, 0.2, 0.2, -0.2);

                    telemetry.addData("success 1", "sucess");
                    telemetry.update();
                    right = false;
                }
                if(detect1.getcX() > properCX + 5 && left) {
                    // strafe to the left (change fr and bl)
                    drive.goToPosition(0.2, -0.2, -0.2, 0.2);

    //                drive.goToPosition(-0.15, 0.15, 0.15, -0.15);

                    telemetry.addData("success 2", "sucess");
                    telemetry.update();
                    left = false;
                }
    //            if (detect1.getcX() >= properCX - 5 && detect1.getcX() <= properCX + 5) {
    //                drive.goToPosition(0, 0, 0, 0);
    //            }
            }

            drive.goToPosition(0, 0, 0, 0);
            // scoring cone
            scoreCone(184, 165, 163, 147);

            //turn back toward cones
            drive.turn(-Math.PI / 2);

            //find tape, get cone
            useColorSensor();



     */
        } else {
            cont.set(true);
            //lift claw a little bit
            score.goToPosition(50, 0.8);
            sleep(100);
            // go forward next to pole
            drive.goToPositionPID(1060, "go forward next to pole");
            // turn to left 45 degrees to medium pole
            drive.turn45(-Math.PI / 4);
            // go to pole a bit
            drive.goToPosition(0.4, 0.4, 0.4, 0.4, 90, "go forward some to pole");
            sleep(50);

            boolean right = true;
            boolean left = true;

            // camera position correction
            while (detect1.getcX() < properCX - 5 || detect1.getcX() > properCX + 5) {
                telemetry.addData("success", "sucess");
                telemetry.update();
                if (detect1.getcX() < properCX - 5 && right) {
                    // strafe to the right
                    //                drive.goToPosition(0.15, -0.15, -0.15, 0.15);
                    drive.goToPosition(-0.2, 0.2, 0.2, -0.2);

                    telemetry.addData("success 1", "sucess");
                    telemetry.update();
                    right = false;
                }
                if (detect1.getcX() > properCX + 5 && left) {
                    // strafe to the left (change fr and bl)
                    drive.goToPosition(0.2, -0.2, -0.2, 0.2);

                    //                drive.goToPosition(-0.15, 0.15, 0.15, -0.15);

                    telemetry.addData("success 2", "sucess");
                    telemetry.update();
                    left = false;
                }
                //            if (detect1.getcX() >= properCX - 5 && detect1.getcX() <= properCX + 5) {
                //                drive.goToPosition(0, 0, 0, 0);
                //            }
            }

            drive.goToPosition(0, 0, 0, 0);


            scoreConeMed(438, 416, 437, 426);

            // turn back straight
            drive.turn45(Math.PI / 4);

            score.setClawPosition(constants.getClawOpenPos());


            //go forward to blue cone tape adjacent mat
            drive.goToPosition(0.4, 0.4, 0.4, 0.4, drive.avgPosition(828, 856, 941, 926), "go forward to next mat");




            // +'s ---------------------------------------->







            score.setClawPosition(constants.getClawOpenPos());
            score.setCamPosition(constants.getStrafeLowCamPos());
            // turn to tape/cones
            drive.turn90(Math.PI / 2);




            // find tape, get cone
            useColorSensor();
            sleep(50);
            drive.goToPosition(-0.5, -0.5, -0.5, -0.5, 500, "go backwards");

            drive.turn90(Math.PI / 2);
            score.goToPosition(constants.getHeightLow(), 0.85);

            sleep(50);
            right = true;
            left = true;

            // camera position correction
            while (detect1.getcX() < properCXLow - 5 || detect1.getcX() > properCXLow + 5) {
                telemetry.addData("success", "sucess");
                telemetry.update();
                if (detect1.getcX() < properCXLow - 5 && right) {
                    // strafe to the right
                    //                drive.goToPosition(0.15, -0.15, -0.15, 0.15);
                    drive.goToPosition(-0.2, 0.2, 0.2, -0.2);

                    telemetry.addData("success 1", "sucess");
                    telemetry.update();
                    right = false;
                }
                if (detect1.getcX() > properCXLow + 5 && left) {
                    // strafe to the left (change fr and bl)
                    drive.goToPosition(0.2, -0.2, -0.2, 0.2);

                    //                drive.goToPosition(-0.15, 0.15, 0.15, -0.15);

                    telemetry.addData("success 2", "sucess");
                    telemetry.update();
                    left = false;
                }
                //            if (detect1.getcX() >= properCX - 5 && detect1.getcX() <= properCX + 5) {
                //                drive.goToPosition(0, 0, 0, 0);
                //            }
            }

            drive.goToPosition(0, 0, 0, 0);

            scoreConeLow(150, 150, 150, 150);



            drive.resetIMU();


            //next cone ------------------------------------------------------>


            drive.turn90(-Math.PI / 2);


            // find tape, get cone
            drive.findTapeMulti("blue");
            //makes sure actually turned 90 degrees
            if(Math.abs(drive.getAngularOrientation() - Math.PI / 2) > 0.15){
                drive.turn(-Math.PI/2);
            }

            score.goToPosition(constants.getStackHeight()-constants.getStackIntervalHeight(), 0.7);

            score.grabConeAuto();

            sleep(100);
            drive.goToPosition(-0.5, -0.5, -0.5, -0.5, 500, "go backwards");

            drive.turn90(Math.PI / 2);

            score.goToPosition(constants.getHeightLow(), 0.85);
            sleep(50);

            right = true;
            left = true;

            // camera position correction
            while (detect1.getcX() < properCXLow - 5 || detect1.getcX() > properCXLow + 5) {
                telemetry.addData("success", "sucess");
                telemetry.update();
                if (detect1.getcX() < properCXLow - 5 && right) {
                    // strafe to the right
                    //                drive.goToPosition(0.15, -0.15, -0.15, 0.15);
                    drive.goToPosition(-0.2, 0.2, 0.2, -0.2);

                    telemetry.addData("success 1", "sucess");
                    telemetry.update();
                    right = false;
                }
                if (detect1.getcX() > properCXLow + 5 && left) {
                    // strafe to the left (change fr and bl)
                    drive.goToPosition(0.2, -0.2, -0.2, 0.2);

                    //                drive.goToPosition(-0.15, 0.15, 0.15, -0.15);

                    telemetry.addData("success 2", "sucess");
                    telemetry.update();
                    left = false;
                }
                //            if (detect1.getcX() >= properCX - 5 && detect1.getcX() <= properCX + 5) {
                //                drive.goToPosition(0, 0, 0, 0);
                //            }
            }

            drive.goToPosition(0, 0, 0, 0);

            scoreConeLow(150, 150, 150, 150);

            sleep(50);

            // PARK ------------------------------------------------------>

            //moves robot to correct parking position
                if (detect1.getParkPosition() == ContourMultiScoreLeft.ParkingPosition.LEFT) {

                    //strafes right to left parkign pos
                    drive.goToPosition(0.35, -0.35, -0.35, 0.35, 400, "strafe right");
                    //moves forward a bit
                    drive.goToPosition(0.3, 0.3, 0.3, 0.3 , 400, "move forward");

                } else if (detect1.getParkPosition() == ContourMultiScoreLeft.ParkingPosition.CENTER) {
                    // move to center park (strafes left)
                    drive.goToPosition(-0.35, 0.35, 0.35, -0.35, 400, "strafe right");
                    //moves forward a bit
                    drive.goToPosition(0.3, 0.3, 0.3, 0.3 , 400, "move forward");

                } else {
                    // turn and move forward to the other pos
                    drive.turn90(Math.PI/2);
                    drive.goToPositionPID(1000, "hi");
                    drive.goToPosition(-0.3, 0.3, 0.3, -0.3 , 200, "strafe right");


                }


        }

    }

    public void scoreConeMed(int fl, int fr, int bl, int br) {
        cont.set(true);
        // move arm medium
        score.goToPosition(constants.getHeightMed(), 0.85);
        //begin thread for maintaining height of slides


        //3700 - 3800
        drive.goToPosition(0.15, 0.15, 0.15, 0.15);
        sleep(100);
        while (detect1.getBoundArea() <= 8000.0 || detect1.getBoundArea() >= 9600) {
            if (detect1.getBoundArea() >= 8000.0 && detect1.getBoundArea() <= 9600 && detect1.getDistance() <= 6/*|| detect1.getcX() <= 18*/) {
                drive.goToPosition(0, 0, 0, 0);
            }

        }



        drive.goToPosition(0, 0, 0, 0);


        sleep(100);
        cont.set(false);
        //lower cone onto pole
        score.goToPosition(score.getEncoderPosition()-300, 0.4);
        score.setClawPosition(constants.getClawOpenPos());
        sleep(100);

        //move back from pole
        drive.goToPosition(-0.3, -0.3, -0.3, -0.3, drive.avgPosition(fl, fr, bl, br), "move back from pole");
        //moves slides down
        score.goToPosition(0, 0.6);
        sleep(100);
    }

    public void scoreConeLow(int fl, int fr, int bl, int br) {
        cont.set(true);
        // move arm medium
        score.goToPosition(constants.getHeightLow(), 0.85);
        //begin thread for maintaining height of slides



        //3700 - 3800
        drive.goToPosition(0.15, 0.15, 0.15, 0.15);
        sleep(100);
        while (detect1.getBoundArea() <= 8500.0 || detect1.getBoundArea() >= 10000) {
            if (detect1.getBoundArea() >= 8500.0 && detect1.getBoundArea() <= 10000 && detect1.getDistance() <= 4.2/*|| detect1.getcX() <= 18*/) {
                drive.goToPosition(0, 0, 0, 0);
            }

        }



        drive.goToPosition(0, 0, 0, 0);


        sleep(500);
        cont.set(false);
        //lower cone onto pole
        score.goToPosition(score.getEncoderPosition()-300, 0.4);
        score.setClawPosition(constants.getClawOpenPos());
        sleep(100);

        //move back from pole
        drive.goToPosition(-0.3, -0.3, -0.3, -0.3, drive.avgPosition(fl, fr, bl, br), "move back from pole");
        //moves slides down
        score.goToPosition(0, 0.6);
        sleep(100);
    }

    public void useColorSensor() {
        drive.findTape("blue");
        //makes sure actually turned 90 degrees
        if(Math.abs(drive.getAngularOrientation() - Math.PI / 2) > 0.15){
            drive.turn(Math.PI/2);
        }

        score.goToPosition(constants.getStackHeight()+15, 0.7);

        score.grabConeAuto();


    }
}
