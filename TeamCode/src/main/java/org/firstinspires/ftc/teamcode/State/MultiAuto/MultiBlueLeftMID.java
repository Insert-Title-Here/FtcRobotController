package org.firstinspires.ftc.teamcode.State.MultiAuto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.State.Auto.ContourMultiScoreLeft;
import org.firstinspires.ftc.teamcode.State.Common.Constants;
import org.firstinspires.ftc.teamcode.State.Common.MecanumDrive;
import org.firstinspires.ftc.teamcode.State.Common.ScoringSystem;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.atomic.AtomicBoolean;

@Autonomous
@Config
public class MultiBlueLeftMID extends LinearOpMode {
    // instantiating
    Thread liftThread;
    MecanumDrive drive;
    ContourMultiScoreLeft detect1;
    ScoringSystem score;
    Constants constants;
    OpenCvWebcam webcam;
    AtomicBoolean liftTurn, liftCone;

    // ftc dashboard values + properCX
    private double properCX = 170; //67
    private double properCXHigh = 163; //67

    private double properCXLow = 180; //160  163

    public static int positive_negative = 1;
    public static int turnDenom = 4;
    public static int toggle = -1;
    volatile boolean earlyPark = false;
    volatile boolean lifting = false;
    long time;
    long onTimeout;

    private int sum = -13;




    @Override
    public void runOpMode() throws InterruptedException {
        // value initializing
        detect1 = new ContourMultiScoreLeft(telemetry);
        drive = new MecanumDrive(hardwareMap, telemetry);
        //sets Auto driving to break instead of float
        drive.mecanumDriveAuto(hardwareMap, telemetry);
        score = new ScoringSystem(hardwareMap, telemetry);
        constants = new Constants();
        liftTurn = new AtomicBoolean();
        liftCone = new AtomicBoolean();
        liftCone.set(false);
        liftTurn.set(false);


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
                    if((score.getEncoderPosition() > 30) && !lifting){
                        score.setPower(constants.getSteadyPow());
                    }
                    if(liftTurn.get()){
                        //score.stackDown();
                        score.goToPosition(constants.getStackHeight()-sum-15, 0.6);
                        liftTurn.set(false);
                    }
                    if (liftCone.get()) {
                        drive.goToPosition(0.4,0.4, 0.4, 0.4, 15, "back a bit");
                    }
                    if((System.currentTimeMillis()-time)/1000 > 24){
                        earlyPark = true;
                    }

//
                 telemetry.addData("imu", drive.currentAngle());
                   telemetry.update();
                }

            }
        };

        //properCX = detect1.getBoundWidth();
        // code to turn servo of cam
        //score.setCamPosition(constants.getSleeveCamPos());
        //detect1.park = true;

        // ftc dashboard setup
        FtcDashboard.getInstance().startCameraStream(webcam, 0);
        telemetry.addData("Status", "Initialized");

        telemetry.update();

        //close claw
        score.setClawPosition(constants.getClawClosePos()); //TODO

        // turn servo of cam forward for sleeves
        score.setCamPosition(constants.getSleeveCamPos());


        waitForStart();
        detect1.park = false;
        // turn servo of cam forward for poles
        //score.setCamPosition(constants.getStrafeLowCamPos());

        liftThread.start();
        time = System.currentTimeMillis();
        //detect1.park = false;



        if (toggle == 0) {
            //lift claw a little bit
            score.goToPosition(50, 0.8);
            sleep(100);
            // go forward next to pole
            drive.goToPosition(0.8, 0.8, 0.8, 0.8, 1060, "go forward next to pole");
            // turn to left 45 degrees to medium pole
            drive.turn45(-Math.PI / 4);
            // go to pole a bit
            drive.goToPosition(0.4, 0.4, 0.4, 0.4, 90, "go forward some to pole");
            sleep(50);

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
        } else if (toggle == 1) {
            score.setClawPosition(constants.getClawOpenPos());
            score.setCamPosition(constants.getSleeveCamPos());
            drive.findTapeMIDAUTO("blueleft");

            //move arm to stack height
            lifting = true;
            score.goToPosition(constants.getStackHeight()-15, 0.5);
            lifting = false;

            //makes sure actually turned 90 degrees
            if(Math.abs(drive.getAngularOrientation() - 0) > 0.15){
                drive.turn(0);
            }



            score.grabConeAuto();


            sleep(50);
            drive.goToPosition(-0.4, -0.4, -0.4, -0.4, 1580, "go backwards");

            drive.absTurnPID(Math.PI / 2);

            lifting = true;
            score.goToPositionPID(constants.getHeightMed()+30, 0.85);
            lifting = false;

            boolean right = true;
            boolean left = true;

            // camera position correction
            while (detect1.getcX() < properCX - 5 || detect1.getcX() > properCX + 5) {
                telemetry.addData("success", "sucess");
                telemetry.update();
                if (detect1.getcX() < properCX - 5 && right) {
                    // strafe to the right
                    //                drive.goToPosition(0.15, -0.15, -0.15, 0.15);
                    drive.goToPosition(-0.25, 0.25, 0.25, -0.25);

                    telemetry.addData("success 1", "sucess");
                    telemetry.update();
                    right = false;
                }
                if (detect1.getcX() > properCX + 5 && left) {
                    // strafe to the left (change fr and bl)
                    drive.goToPosition(0.25, -0.25, -0.25, 0.25);

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

            scoreConeMed(100, 150, 100, 100);

            drive.absTurnPID(0);

            drive.goToPosition(0.4, 0.4, 0.4, 0.4, 1140, "go forwards");

            drive.goToPosition(0.8, 0, 0, 0.8, 75, "to the right a bit");


            drive.findTapeMIDAUTO("blueleft");

            //move arm to stack height
            lifting = true;
            score.goToPosition(constants.getStackHeight() - constants.getStackIntervalHeight(), 0.8);
            lifting = false;



            // ------------------------------------------------------------------

            //makes sure actually turned 90 degrees
            if(Math.abs(drive.getAngularOrientation() - 0) > 0.15) {
                drive.turn(0);
            }

            score.grabConeAuto();


            sleep(50);
            drive.goToPosition(-0.4, -0.4, -0.4, -0.4, 1580, "go backwards");

            drive.absTurnPID(Math.PI / 2);

            lifting = true;
            score.goToPositionPID(constants.getHeightMed()+30, 0.85);
            lifting = false;

             right = true;
             left = true;

            // camera position correction
            while (detect1.getcX() < properCXHigh - 5 || detect1.getcX() > properCXHigh + 5) {
                telemetry.addData("success", "sucess");
                telemetry.update();
                if (detect1.getcX() < properCX - 5 && right) {
                    // strafe to the right
                    //                drive.goToPosition(0.15, -0.15, -0.15, 0.15);
                    drive.goToPosition(-0.25, 0.25, 0.25, -0.25);

                    telemetry.addData("success 1", "sucess");
                    telemetry.update();
                    right = false;
                }
                if (detect1.getcX() > properCX + 5 && left) {
                    // strafe to the left (change fr and bl)
                    drive.goToPosition(0.25, -0.25, -0.25, 0.25);

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

            scoreConeMed(100, 150, 100, 100);

            drive.absTurnPID(0);
        } else if (toggle == 2) {
            drive.goToPosition(0, 0.3, 0.3, 0, 200, "1");

        } else {

            //lift claw a little bit
            score.goToPosition(50, 0.8);
            sleep(100);
            // go forward next to pole
            drive.goToPosition(0.8, 0.8, 0.8, 0.8, 2150, "go forward next to pole");
            // turn to right 45 degrees to high pole
            drive.turn45(-Math.PI / 4);
            // go to pole a bit
            drive.goToPosition(0.4, 0.4, 0.4, 0.4, 90, "go forward some to pole");
            // move arm high
            lifting = true;
            score.goToPositionPID(constants.getHeightHigh()+5, 0.85);
            lifting = false;
            sleep(50);

            boolean right = true;
            boolean left = true;

            // camera position correction
            while (detect1.getcX() < properCXHigh - 5 || detect1.getcX() > properCXHigh + 5) {
                telemetry.addData("success", "sucess");
                telemetry.update();
                if (detect1.getcX() < properCXHigh - 5 && right) {
                    // strafe to the right
                    //                drive.goToPosition(0.15, -0.15, -0.15, 0.15);
                    drive.goToPosition(-0.25, 0.25, 0.25, -0.25);

                    telemetry.addData("success 1", "sucess");
                    telemetry.update();
                    right = false;
                }
                if (detect1.getcX() > properCXHigh + 5 && left) {
                    // strafe to the left (change fr and bl)
                    drive.goToPosition(0.25, -0.25, -0.25, 0.25);

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

            sum = 0;
            scoreConeMed(300, 0, 0, 0);

            // turn to left
//            drive.turn90(Math.PI / 4 + Math.PI / 1.9);
            drive.absTurnPID(Math.PI / 2.1);

            score.setClawPosition(constants.getClawOpenPos());
            //go forward a bit
            drive.goToPosition(0.6, 0.6, 0.6, 0.6, 300, "forward" );

            drive.goToPosition(0.6, -0.3, -0.3, 0.6, 150, "to the right a bit");


            // +'s ---------------------------------------->

            sum = constants.getStackIntervalHeight();





            score.setCamPosition(constants.getSleeveCamPos());
            drive.findTapeMIDAUTO("blueleft");



            //makes sure actually turned 90 degrees
            if(Math.abs(drive.getAngularOrientation() - Math.PI / 2) > 0.1){
                drive.turn(Math.PI / 1.9);
            }



            score.grabConeAuto();


            sleep(50);
            drive.goToPosition(-0.8, -0.85, -0.8, -0.85, 1380, "go backwards");

            drive.absTurnPID(Math.PI / 1.005);

           lifting = true;
           score.goToPositionPID(constants.getHeightMed()+20, 0.85);
           lifting = false;

             right = true;
             left = true;

            // camera position correction
            while (detect1.getcX() < properCX - 5 || detect1.getcX() > properCX + 5) {
                telemetry.addData("success", "sucess");
                telemetry.update();
                if (detect1.getcX() < properCX - 5 && right) {
                    // strafe to the right
                    //                drive.goToPosition(0.15, -0.15, -0.15, 0.15);
                    drive.goToPosition(-0.25, 0.25, 0.25, -0.25);

                    telemetry.addData("success 1", "sucess");
                    telemetry.update();
                    right = false;
                }
                if (detect1.getcX() > properCX + 5 && left) {
                    // strafe to the left (change fr and bl)
                    drive.goToPosition(0.25, -0.25, -0.25, 0.25);

                    //                drive.goToPosition(-0.15, 0.15, 0.15, -0.15);

                    telemetry.addData("success 2", "sucess");
                    telemetry.update();
                    left = false;
                }
                //            if (detect1.getcX() >= properCX - 5 && detect1.getcX() <= properCX + 5) {
                //                drive.goToPosition(0, 0, 0, 0);
                //            }
            }

            drive.goToPosition(0.5, 0.5, 0.5, 0.5, 50, "forward a bit");

            scoreConeMed(100, 150, 100, 100);

            drive.absTurnPID(Math.PI / 2);

            drive.goToPosition(0.6, 0.6, 0.6, 0.6, 1040, "go forwards");

            drive.goToPosition(0.6, -0.2, -0.2, 0.6, 100, "to the right a bit");


            drive.findTapeMIDAUTO("blueleft");





            // 2nd +' CONE --------------------------------------------------------->
            score.setCamPosition(constants.getSleeveCamPos() + 0.02);

            sum = constants.getStackIntervalHeight() * 2;

            //makes sure actually turned 90 degrees
            if(Math.abs(drive.getAngularOrientation() - Math.PI / 2) > 0.1) {
                drive.turn(Math.PI / 1.9);
            }


            score.grabConeAuto();



            sleep(50);
            drive.goToPosition(-0.8, -0.85, -0.8, -0.85, 1380, "go backwards");

            drive.absTurnPID(Math.PI / 1.005);

            lifting = true;
            score.goToPositionPID(constants.getHeightMed()+30, 0.85);
            lifting = false;

            right = true;
            left = true;

            // camera position correction
            while (detect1.getcX() < properCX - 5 || detect1.getcX() > properCX + 5) {
                telemetry.addData("success", "sucess");
                telemetry.update();
                if (detect1.getcX() < properCX - 5 && right) {
                    // strafe to the right
                    //                drive.goToPosition(0.15, -0.15, -0.15, 0.15);
                    drive.goToPosition(-0.25, 0.25, 0.25, -0.25);

                    telemetry.addData("success 1", "sucess");
                    telemetry.update();
                    right = false;
                }
                if (detect1.getcX() > properCX + 5 && left) {
                    // strafe to the left (change fr and bl)
                    drive.goToPosition(0.25, -0.25, -0.25, 0.25);

                    //                drive.goToPosition(-0.15, 0.15, 0.15, -0.15);

                    telemetry.addData("success 2", "sucess");
                    telemetry.update();
                    left = false;
                }
                //            if (detect1.getcX() >= properCX - 5 && detect1.getcX() <= properCX + 5) {
                //                drive.goToPosition(0, 0, 0, 0);
                //            }
            }

            drive.goToPosition(0.5, 0.5, 0.5, 0.5, 100, "forward a bit");

            scoreConeMed(100, 150, 100, 100);

            drive.absTurnPID(Math.PI / 2);

            drive.goToPosition(0.6, 0.6, 0.6, 0.6, 1040, "go forwards");

            drive.goToPosition(0.6, -0.2, -0.2, 0.6, 200, "to the right a bit");


            drive.findTapeMIDAUTO("blueleft");

            // 3rd +' CONE --------------------------------------------------------->
            score.setCamPosition(constants.getSleeveCamPos() + 0.02*2);
            sum = constants.getStackIntervalHeight() * 3;

            //makes sure actually turned 90 degrees
            if(Math.abs(drive.getAngularOrientation() - Math.PI / 2) > 0.1) {
                drive.turn(Math.PI / 1.9);
            }


            score.grabConeAuto();



            sleep(50);
            drive.goToPosition(-0.8, -0.85, -0.8, -0.85, 1380, "go backwards");

            drive.absTurnPID(Math.PI / 1.005);

            lifting = true;
            score.goToPositionPID(constants.getHeightMed()+30, 0.85);
            lifting = false;

            right = true;
            left = true;

            // camera position correction
            while (detect1.getcX() < properCX - 5 || detect1.getcX() > properCX + 5) {
                telemetry.addData("success", "sucess");
                telemetry.update();
                if (detect1.getcX() < properCX - 5 && right) {
                    // strafe to the right
                    //                drive.goToPosition(0.15, -0.15, -0.15, 0.15);
                    drive.goToPosition(-0.25, 0.25, 0.25, -0.25);

                    telemetry.addData("success 1", "sucess");
                    telemetry.update();
                    right = false;
                }
                if (detect1.getcX() > properCX + 5 && left) {
                    // strafe to the left (change fr and bl)
                    drive.goToPosition(0.25, -0.25, -0.25, 0.25);

                    //                drive.goToPosition(-0.15, 0.15, 0.15, -0.15);

                    telemetry.addData("success 2", "sucess");
                    telemetry.update();
                    left = false;
                }
                //            if (detect1.getcX() >= properCX - 5 && detect1.getcX() <= properCX + 5) {
                //                drive.goToPosition(0, 0, 0, 0);
                //            }
            }

            drive.goToPosition(0.5, 0.5, 0.5, 0.5, 100, "forward a bit");

            scoreConeMed(100, 150, 100, 100);

            drive.absTurnPID(Math.PI / 2);


            //}
            // PARK ------------------------------------------------------>

            //moves robot to correct parking position
                if (detect1.getParkPosition() == ContourMultiScoreLeft.ParkingPosition.LEFT) {
                    // go forward
                    drive.goToPosition(0.6, 0.6, 0.6, 0.6, 1240, "go forwards");
                    // strafe left
                    drive.goToPosition(-0.6, 0.6, 0.6, -0.6, 700, "strafe left");

                } else if (detect1.getParkPosition() == ContourMultiScoreLeft.ParkingPosition.CENTER) {
                    // go forward
                    drive.goToPosition(0.6, 0.6, 0.6, 0.6, 300, "go forwards");
                    // strafe left
                    drive.goToPosition(-0.6, 0.6, 0.6, -0.6, 700, "strafe left");
                } else {
                    // go backwards
                    drive.goToPosition(-0.6, -0.6, -0.6, -0.6, 300, "go backwards");
                    // strafe left
                    drive.goToPosition(-0.6, 0.6, 0.6, -0.6, 700, "strafe left");


                }


        }

    }

    public void scoreConeMed(int fl, int fr, int bl, int br) {
        /*
        // move arm medium
        lifting = true;
        score.goToPositionPID(constants.getHeightMed()+30)+10, 0.85);
        lifting = false;
        */

        //begin thread for maintaining height of slides

        onTimeout = System.currentTimeMillis();
        //3700 - 3800
        drive.goToPosition(0.18, 0.18, 0.18, 0.18);
        sleep(100);
        // old: 7900     (9100 new)  (6800 new)

        while (detect1.getBoundArea() <= 5900.0 || detect1.getBoundArea() >= 7700) { //7200
            if (detect1.getBoundArea() >= 5900.0 && detect1.getBoundArea() <= 7700 && detect1.getDistance() <= 5.5/*|| detect1.getcX() <= 18*/) {
                drive.goToPosition(0, 0, 0, 0);
                break;
            }else if((System.currentTimeMillis()-onTimeout)/1000 > 3){
                break;
            }

        }





        sleep(50);

        //lower cone ontto pole
        liftTurn.set(true);
        sleep(100);
        score.setClawPosition(constants.getClawOpenPos());


        //move back from pole
        drive.goToPosition(-0.4, -0.4, -0.4, -0.4, fl, "move back from pole");

    }

    public void scoreConeLow(int fl, int fr, int bl, int br) {
        drive.goToPosition(0.3, 0.3, 0.3, 0.3, 20, "move forward");

        //begin thread for maintaining height of slides


        onTimeout = System.currentTimeMillis();
        //3700 - 3800
        drive.goToPosition(0.18, 0.18, 0.18, 0.18);
        // prev: 9000      (8700 now)
        while (detect1.getBoundArea() <= 5800.0 || detect1.getBoundArea() >= 7300) { //6600
            if (detect1.getBoundArea() >= 5800.0 && detect1.getBoundArea() <= 7300 && detect1.getDistance() <= 5.3) {
                drive.goToPosition(0, 0, 0, 0);
                break;
            }else if((System.currentTimeMillis()-onTimeout)/1000 > 3) {
                break;
            }


        }



        drive.goToPosition(0, 0, 0, 0);


        sleep(50);

        //lower cone onto pole
        score.goToPosition(score.getEncoderPosition() - 300, 0.7);
        liftTurn.set(true);
        score.setClawPosition(constants.getClawOpenPos());
        sleep(50);


        //move back from pole
        drive.goToPosition(-0.4, -0.4, -0.4, -0.4, drive.avgPosition(fl, fr, bl, br), "move back from pole");

    }

    public void useColorSensor() {
        drive.findTape("blueleft", true);
        //makes sure actually turned 90 degrees
        if(Math.abs(drive.getAngularOrientation() - Math.PI / 2) > 0.15){
            drive.turn(Math.PI/2);
        }

        score.goToPosition(constants.getStackHeight(), 0.7);

        score.grabConeAuto();


    }
}
