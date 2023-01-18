package org.firstinspires.ftc.teamcode.State.MultiAuto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.State.Auto.ContourMultiScoreRight;
import org.firstinspires.ftc.teamcode.State.Common.Constants;
import org.firstinspires.ftc.teamcode.State.Common.MecanumDrive;
import org.firstinspires.ftc.teamcode.State.Common.ScoringSystem;
import org.firstinspires.ftc.teamcode.State.Common.Times;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.atomic.AtomicBoolean;

@Autonomous @Config
public class MultiRedRightLOW extends LinearOpMode {
    // instantiating
    Thread liftThread;
    MecanumDrive drive;
    ContourMultiScoreRight detect1;
    ScoringSystem score;
    Constants constants;
    OpenCvWebcam webcam;
    AtomicBoolean liftTurn, liftCone, timeout;
    public volatile boolean earlyPark;


    // ftc dashboard values + properCX
    private double properCX = 187; //67
    private double properCXLow = 160;

    public static int positive_negative = 1;
    public static int turnDenom = 4;
    public static boolean toggle = false;
    public long currentTime;


    private int sum = -13;




    @Override
    public void runOpMode() throws InterruptedException {
        // value initializing
        detect1 = new ContourMultiScoreRight(telemetry);
        drive = new MecanumDrive(hardwareMap, telemetry, false);
        score = new ScoringSystem(hardwareMap, telemetry);
        constants = new Constants();
        liftTurn = new AtomicBoolean();
        liftCone = new AtomicBoolean();
        timeout = new AtomicBoolean();
        earlyPark = false;
        liftCone.set(false);
        liftTurn.set(false);
        timeout.set(false);


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
                    if((score.getEncoderPosition() > 50)){
                        score.setPower(constants.getSteadyPow());
                    }
                    if(liftTurn.get()){
                        //score.stackDown();
                        score.goToPosition(constants.getStackHeight()-sum-10, 0.6);
                        liftTurn.set(false);
                    }
                    if (liftCone.get()) {
                        drive.goToPosition(0.4,0.4, 0.4, 0.4, 15, "back a bit");
                    }
                    if (Times.timeDifference() > 4) {
                        timeout.set(true);
                    }
                    if((System.currentTimeMillis() - currentTime)/1000 >= 24){
                        earlyPark = true;
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

        // turn servo of cam forward for sleeves
        score.setCamPosition(constants.getSleeveCamPos());


        waitForStart();
        currentTime = System.currentTimeMillis();
        detect1.park = false;
        // turn servo of cam forward for poles
        score.setCamPosition(constants.getStrafeLowCamPos());

        liftThread.start();
        //detect1.park = false;



        if (toggle) {
            //lift claw a little bit
            score.goToPosition(50, 0.8);
            sleep(100);
            // go forward next to pole
            drive.goToPosition(0.8, 0.8, 0.8, 0.8, 900, "go forward next to pole");
            // turn to left 45 degrees to medium pole
            drive.turn45(-Math.PI / 4.7);
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
        } else {


            //lift claw a little bit
            score.goToPosition(50, 0.8);
            sleep(100);
            // go forward next to pole
            drive.goToPosition(0.8, 0.8, 0.8, 0.8, 930, "go forward next to pole");
            // turn to left 45 degrees to medium pole
            drive.turn45(Math.PI / 5.8);
            // go to pole a bit
            drive.goToPosition(0.5, 0.5, 0.5, 0.5, 110, "go forward some to pole");
            sleep(50);

            boolean right = true;
            boolean left = true;

            // camera position correction
            while (detect1.getcX() < properCXLow - 5 || detect1.getcX() > properCXLow + 5) {
                telemetry.addData("success", "sucess");
                telemetry.update();
                if (detect1.getcX() < properCXLow - 5 && right) {
                    // strafe to the right
                    //                drive.goToPosition(0.15, -0.15, -0.15, 0.15);
                    drive.goToPosition(-0.25, 0.25, 0.25, -0.25);

                    telemetry.addData("success 1", "sucess");
                    telemetry.update();
                    right = false;
                }
                if (detect1.getcX() > properCXLow + 5 && left) {
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


            scoreConeMed(500, 500, 300, 300);

            score.goToPosition(0, 0.8);

            // turn back straight
            drive.turn45(-Math.PI / 3.8);

            score.setClawPosition(constants.getClawOpenPos());


            //go forward to blue cone tape adjacent mat
            drive.goToPosition(0.5, 0.5, 0.5, 0.5, drive.avgPosition(1128, 1100, 1041, 1026), "go forward to next mat");
            drive.goToPosition(-0.5, -0.5, -0.5, -0.5, 100, "back");


            //823.75

            // +'s ---------------------------------------->

            sum = constants.getStackIntervalHeight()-20;





            score.setClawPosition(constants.getClawOpenPos());
            score.setCamPosition(constants.getStrafeLowCamPos());

            // turn to tape/cones
            drive.turn90(-Math.PI / 1.95);

            score.goToPosition(constants.getStackHeight()-sum, 0.8);



            // find tape, get cone
            drive.findTape("blueright", true);
            //makes sure actually turned 90 degrees
            if(Math.abs(drive.getAngularOrientation() + Math.PI / 2) >0.08){
                drive.turn(-Math.PI/2);
            }



            score.grabConeAuto();


            sleep(50);
            drive.goToPosition(-0.4, -0.4, -0.4, -0.4, 500, "go backwards");

            drive.turn90(-Math.PI / 1.9);


            right = true;
            left = true;

            // camera position correction
            while (detect1.getcX() < properCXLow - 5 || detect1.getcX() > properCXLow + 5) {
                telemetry.addData("success", "sucess");
                telemetry.update();
                if (detect1.getcX() < properCXLow - 5 && right) {
                    // strafe to the right
                    //                drive.goToPosition(0.15, -0.15, -0.15, 0.15);
                    drive.goToPosition(-0.25, 0.25, 0.25, -0.25);

                    telemetry.addData("success 1", "sucess");
                    telemetry.update();
                    right = false;
                }
                if (detect1.getcX() > properCXLow + 5 && left) {
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

            scoreConeLow(90, 90, 90, 150);


            //next cone ------------------------------------------------------>

            sum = constants.getStackIntervalHeight()*2-10;


            if(drive.getAngularOrientation() > 0){
                drive.turn90(Math.PI / 4);
                drive.turn90(Math.PI / 2);
            }else if(drive.getAngularOrientation() < 0){
                drive.turn90(Math.PI / 2);
            }


            // find tape, get cone
            drive.findTapeMulti("blueright");
            //makes sure actually turned 90 degrees
            if(Math.abs(drive.getAngularOrientation() + Math.PI / 2) > 0.08){
                drive.turn(-Math.PI/2);
            }



            score.grabConeAuto();

            sleep(100);
            drive.goToPosition(-0.4, -0.4, -0.4, -0.4, 500, "go backwards");

            drive.turn90(-Math.PI / 2);


            right = true;
            left = true;

            // camera position correction
            while (detect1.getcX() < properCXLow-2 - 5 || detect1.getcX() > properCXLow-2 + 5) {
                telemetry.addData("success", "sucess");
                telemetry.update();
                if (detect1.getcX() < properCXLow-2 - 5 && right) {
                    // strafe to the right
                    //                drive.goToPosition(0.15, -0.15, -0.15, 0.15);
                    drive.goToPosition(-0.25, 0.25, 0.25, -0.25);

                    telemetry.addData("success 1", "sucess");
                    telemetry.update();
                    right = false;
                }
                if (detect1.getcX() > properCXLow-2 + 5 && left) {
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

            scoreConeLow(90, 90, 90, 150);

            sleep(50);

            if(!earlyPark) {
                // 3rd +' CONE --------------------------------------------------------->
                sum = constants.getStackIntervalHeight() * 3 - 10;


                if (drive.getAngularOrientation() > 0) {
                    drive.turn90(Math.PI / 4);
                    drive.turn90(Math.PI / 2);
                } else if (drive.getAngularOrientation() < 0) {
                    drive.turn90(Math.PI / 2);
                }


                // find tape, get cone
                drive.findTapeMulti("blueright");
                //makes sure actually turned 90 degrees
                if (Math.abs(drive.getAngularOrientation() + Math.PI / 2) >0.08) {
                    drive.turn(-Math.PI / 2);
                }


                score.grabConeAuto();

                sleep(100);
                drive.goToPosition(-0.4, -0.4, -0.4, -0.4, 500, "go backwards");

                drive.turn90(-Math.PI / 2);


                right = true;
                left = true;

                // camera position correction
                while (detect1.getcX() < properCXLow - 4 - 5 || detect1.getcX() > properCXLow - 4 + 5) {
                    telemetry.addData("success", "sucess");
                    telemetry.update();
                    if (detect1.getcX() < properCXLow - 4 - 5 && right) {
                        // strafe to the right
                        //                drive.goToPosition(0.15, -0.15, -0.15, 0.15);
                        drive.goToPosition(-0.25, 0.25, 0.25, -0.25);

                        telemetry.addData("success 1", "sucess");
                        telemetry.update();
                        right = false;
                    }
                    if (detect1.getcX() > properCXLow - 4 + 5 && left) {
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

                scoreConeLow(90, 90, 90, 150);

                sleep(50);
            }
            // PARK ------------------------------------------------------>

            //moves robot to correct parking position
            if (detect1.getParkPosition() == ContourMultiScoreRight.ParkingPosition.LEFT) {
                // turn and move forward to the other pos
                drive.turn90(-Math.PI/2.25);
                drive.goToPosition(1, 1, 1, 1,1400, "forward to 3rd park place");
                drive.turn45(Math.PI / 8);
                drive.goToPosition(-0.8, 0.8, 0.8, +0.8 , 350, "strafe left");



            } else if (detect1.getParkPosition() == ContourMultiScoreRight.ParkingPosition.CENTER) {
                // move to center park (strafes right)
                drive.goToPosition(0.8, -0.8, -0.8, 0.8, 500, "strafe right");
                //moves forward a bit
                drive.goToPosition(0.3, 0.3, 0.3, 0.3 , 400, "move forward");

            } else {
                //strafes left to left parkign pos
                drive.goToPosition(-0.9, 0.9, 0.9, -0.9, 490, "strafe left");
                //moves forward a bit
                drive.goToPosition(0.3, 0.3, 0.3, 0.3 , 400, "move forward");

            }


        }

    }

    public void scoreConeMed(int fl, int fr, int bl, int br) {

        // move arm medium
        score.goToPosition(constants.getHeightMed(), 0.85);
        //begin thread for maintaining height of slides


        //3700 - 3800
        drive.goToPosition(0.18, 0.18, 0.18, 0.18);
        Times.startTime();
        while (detect1.getBoundArea() <= 7900.0 || detect1.getBoundArea() >= 9600 && !timeout.get()) {
            if (detect1.getBoundArea() >= 7900.0 && detect1.getBoundArea() <= 9600 && detect1.getDistance() <= 6/*|| detect1.getcX() <= 18*/) {
                drive.goToPosition(0, 0, 0, 0);
            }

        }
        timeout.set(false);




        sleep(50);

        //lower cone onto pole
        liftTurn.set(true);
        score.setClawPosition(constants.getClawOpenPos());



        //move back from pole
        drive.goToPosition(-0.4, -0.4, -0.4, -0.4, drive.avgPosition(fl, fr, bl, br), "move back from pole");

    }

    public void scoreConeLow(int fl, int fr, int bl, int br) {
        drive.goToPosition(0.3, 0.3, 0.3, 0.3, 20, "move forward");

        //begin thread for maintaining height of slides



        //3700 - 3800
        drive.goToPosition(0.18, 0.18, 0.18, 0.18);
        Times.startTime();
        while (detect1.getBoundArea() <= 9000.0 || detect1.getBoundArea() >= 10500 && !timeout.get()) {
            if (detect1.getBoundArea() >= 9000.0 && detect1.getBoundArea() <= 10500 && detect1.getDistance() <= 4) {
                drive.goToPosition(0, 0, 0, 0);
            }

        }
        timeout.set(false);



        drive.goToPosition(0, 0, 0, 0);


        sleep(50);

        //lower cone onto pole
        score.goToPosition(score.getEncoderPosition() - 300, 0.8);
        liftTurn.set(true);
        score.setClawPosition(constants.getClawOpenPos());
        sleep(50);


        //move back from pole
        drive.goToPosition(-0.4, -0.4, -0.4, -0.4, drive.avgPosition(fl, fr, bl, br), "move back from pole");

    }

    public void useColorSensor() {
        drive.findTape("blueright", true);
        //makes sure actually turned 90 degrees
        if(Math.abs(drive.getAngularOrientation() + Math.PI / 2) > 0.08){
            drive.turn(-Math.PI/2);
        }

        score.goToPosition(constants.getStackHeight(), 0.7);

        score.grabConeAuto();


    }
}
