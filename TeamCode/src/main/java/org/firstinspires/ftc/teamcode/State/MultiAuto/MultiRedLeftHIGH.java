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
public class MultiRedLeftHIGH extends LinearOpMode {
    // instantiating
    Thread liftThread;
    MecanumDrive drive;
    ContourMultiScoreLeft detect1;
    ScoringSystem score;
    Constants constants;
    OpenCvWebcam webcam;
    AtomicBoolean liftTurn, liftCone, safePark;

    // ftc dashboard values + properCX
    private double properCX = 165; //67  170
    private double properCXHigh = 160   ; //67

    private double properCXLow = 180; //160  163

    public static int positive_negative = 1;
    public static int turnDenom = 4;
    public static int toggle = -1;
    public static int inner_toggle = 0;
    private boolean toggleIFS = true;
    volatile boolean earlyPark = false;
    volatile boolean lifting = false;
    volatile boolean grabbingCone = false;
    volatile boolean findingTape = false;
    volatile boolean scoringPole = false;
    volatile boolean strafing = false;


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
        safePark = new AtomicBoolean();
        liftCone.set(false);
        liftTurn.set(false);
        safePark.set(false);


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
            public void run() {
                while (opModeIsActive()) {
                    if ((score.getEncoderPosition() > 30) && !lifting) {
                        score.setPower(constants.getSteadyPow());
                    }
                    if (liftTurn.get()) {
                        //score.stackDown();
                        score.goToPosition(constants.getStackHeight() - sum - 15, 0.6);
                        liftTurn.set(false);
                    }
                    if (liftCone.get()) {
                        drive.goToPosition(0.4, 0.4, 0.4, 0.4, 15, "back a bit");
                    }
                    if ((System.currentTimeMillis() - time) / 1000 > 24) {
                        earlyPark = true;
                    }

                    if (safePark.get()) {
                        long time = System.currentTimeMillis();
                        while (grabbingCone) {

                            if ((System.currentTimeMillis() - time) / 1000 > 2.5) {
                                safeParking(true, false, false, false);

                            }
                        }
                        while (findingTape) {
                            if ((System.currentTimeMillis() - time) / 1000 > 2.5) {
                                safeParking(false, true, false, false);

                            }
                        }
                        while (scoringPole) {
                            if ((System.currentTimeMillis() - time) / 1000 > 3) {
                                safeParking(false, false, true, false);

                            }
                        }
                        while (strafing) {
                            if ((System.currentTimeMillis() - time) / 1000 > 1.5) {
                                safeParking(false, false, false, true);

                            }
                        }
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
            drive.goToPosition(0.8, 0.8, 0.8, 0.8, 758, "go forward next to pole");
            // turn to left 45 degrees to medium pole
            drive.turn45(-Math.PI / 4);
            // go to pole a bit
            drive.goToPosition(0.4, 0.4, 0.4, 0.4, 64, "go forward some to pole");
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



        } else if (toggle == 2) {
            while (toggle == 2) {
                if (inner_toggle == 0) {
                    drive.goToPosition(0, 0, 0, 0);
                    toggleIFS = true;
                } else if (inner_toggle == 1 && toggleIFS) {
                    drive.goToPosition(0.62, 0.6, 0.62, 0.6);
                    toggleIFS = false;
                } else if (inner_toggle == 2 && toggleIFS){
                    drive.goToPosition(-0.8, -0.83, -0.8, -0.83);
                    toggleIFS = false;

                }
            }

        } else if (toggle == 3) {
            while (toggle == 2) {
                if (inner_toggle == 0) {
                    drive.goToPosition(0, 0, 0, 0);
                    toggleIFS = true;
                } else if (inner_toggle == 1 && toggleIFS) {
                    drive.goToPosition(0.62, 0.6, 0.62, 0.6);
                    toggleIFS = false;
                } else if (inner_toggle == 2 && toggleIFS){
                    drive.goToPosition(-0.8, -0.83, -0.8, -0.83);
                    toggleIFS = false;

                }
            }

        }else {

            //lift claw a little bit
            score.goToPosition(50, 0.8);
            sleep(100);
            // go forward next to pole
            drive.goToPosition(0.74, 0.7, 0.74, 0.7, 1350, "go forward next to pole");
            // turn to right 45 degrees to high pole
            drive.absTurnDriftPID(-Math.PI / 4);
//            // go to pole a bit
//            drive.goToPosition(0.52, 0.5, 0.52, 0.5, 25, "go forward some to pole");
            // move arm high
            lifting = true;
            score.goToPositionPID(constants.getHeightHigh()-10, 0.85);
            lifting = false;
            sleep(50);

            boolean right = true;
            boolean left = true;
            safePark.set(true);
            strafing = true;



            // camera position correction
            while (detect1.getcX() < properCXHigh - 5 || detect1.getcX() > properCXHigh + 5) {
                telemetry.addData("success", "sucess");
                telemetry.update();
                if (detect1.getcX() < properCXHigh - 5 && right) {
                    // strafe to the right
                    //                drive.goToPosition(0.15, -0.15, -0.15, 0.15);
                    drive.goToPosition(-0.265, 0.265, 0.265, -0.265);


                    telemetry.addData("success 1", "sucess");
                    telemetry.update();
                    right = false;
                }
                if (detect1.getcX() > properCXHigh + 5 && left) {
                    // strafe to the left (change fr and bl)
                    drive.goToPosition(0.265, -0.265, -0.265, 0.265);


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

            safePark.set(false);
            strafing = false;


            sum = 15;
            scoreConePreload(150, 0, 0, 0);
            drive.goToPosition(-0.3, -0.3, -0.3, -0.3, 100, "back up a little");
            // turn to left
//            drive.turn90(Math.PI / 4 + Math.PI / 1.9);
            drive.absTurnPID(Math.PI / 2);

            score.setClawPosition(constants.getClawOpenPos());
            //go forward a bit
            drive.goToPosition(0.52, 0.5, 0.52, 0.5, 220, "forward");

            drive.goToPosition(-0.25, 0.5, 0.5, -0.25, 110, "to the left a bit");


            // +'s ---------------------------------------->

            sum = constants.getStackIntervalHeight();


            score.setCamPosition(constants.getSleeveCamPos());

            safePark.set(true);
            findingTape = true;
            drive.findTapeMulti("redleft");

            safePark.set(false);
            findingTape = false;


            //makes sure actually turned 90 degrees
            if (Math.abs(drive.getAngularOrientation() - Math.PI / 1.88) > 0.05) {
                drive.turn(Math.PI / 1.88);
            }

            safePark.set(true);
            grabbingCone = true;
            score.grabConeAuto();
            safePark.set(false);
            grabbingCone = false;


            drive.goToPosition(-0.5, 0.5, 0.5, -0.5, 35, "go to left a bit");

            sleep(50);
            drive.goToPosition(-0.5, -0.53, -0.5, -0.53, 937, "go backwards");

            drive.absTurnDriftPID(0);

            lifting = true;
            score.goToPositionPID(constants.getHeightHigh()-30, 0.85);
            lifting = false;

            right = true;
            left = true;
            safePark.set(true);
            strafing = true;

            // camera position correction
            while (detect1.getcX() < properCX - 5 || detect1.getcX() > properCX + 5) {
                telemetry.addData("success", "sucess");
                telemetry.update();
                if (detect1.getcX() < properCX - 5 && right) {
                    // strafe to the right
                    //                drive.goToPosition(0.15, -0.15, -0.15, 0.15);
                    drive.goToPosition(-0.265, 0.265, 0.265, -0.265);

                    telemetry.addData("success 1", "sucess");
                    telemetry.update();
                    right = false;
                }
                if (detect1.getcX() > properCX + 5 && left) {
                    // strafe to the left (change fr and bl)
                    drive.goToPosition(0.265, -0.265, -0.265, 0.265);


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
            safePark.set(false);
            strafing = false;
//            drive.goToPosition(0.52, 0.5, 0.52, 0.5, 36, "forward a bit");



            safePark.set(true);
            scoringPole = true;
            scoreConeMulti(30, 150, 100, 100);

            safePark.set(false);
            scoringPole = false;


            drive.absTurnPID(Math.PI / 2);

            drive.goToPosition(0.52, 0.5, 0.52, 0.5, 720, "go forwards");

            drive.goToPosition(-0.25, 0.5, 0.5, -0.25, 120, "to the left a bit");

            safePark.set(true);
            findingTape = true;
            drive.findTapeMulti("redleft");

            safePark.set(false);
            findingTape = false;


            // 2nd +' CONE --------------------------------------------------------->


            score.setCamPosition(constants.getSleeveCamPos() + 0.02);

            sum = constants.getStackIntervalHeight() * 2;

            //makes sure actually turned 90 degrees
            if (Math.abs(drive.getAngularOrientation() - Math.PI / 1.95) > 0.1) {
//                drive.turn(Math.PI / 1.95);
                drive.absTurnPID(Math.PI / 1.95);
            }

            safePark.set(true);
            grabbingCone = true;
            score.grabConeAuto();
            safePark.set(false);
            grabbingCone = false;



            drive.goToPosition(-0.4, 0.4, 0.4, -0.4, 35, "go to left a bit");


            sleep(50);
            drive.goToPosition(-0.5, -0.53, -0.5, -0.53, 937, "go backwards");

            drive.absTurnDriftPID(0);


            lifting = true;
            score.goToPositionPID(constants.getHeightHigh()-30, 0.85);
            lifting = false;

            right = true;
            left = true;
            safePark.set(true);
            strafing = true;

            // camera position correction
            while (detect1.getcX() < properCX - 5 || detect1.getcX() > properCX + 5) {
                telemetry.addData("success", "sucess");
                telemetry.update();
                if (detect1.getcX() < properCX - 5 && right) {
                    // strafe to the right
                    //                drive.goToPosition(0.15, -0.15, -0.15, 0.15);
                    drive.goToPosition(-0.265, 0.265, 0.265, -0.265);


                    telemetry.addData("success 1", "sucess");
                    telemetry.update();
                    right = false;
                }
                if (detect1.getcX() > properCX + 5 && left) {
                    // strafe to the left (change fr and bl)
                    drive.goToPosition(0.265, -0.265, -0.265, 0.265);


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
            safePark.set(false);
            strafing = false;
//            drive.goToPosition(0.52, 0.5, 0.52, 0.5, 72, "forward a bit");


            safePark.set(true);
            scoringPole = true;
            scoreConeMulti(30, 150, 100, 100);

            safePark.set(false);
            scoringPole = false;

            drive.absTurnPID(Math.PI / 2);


            //}
            // PARK ------------------------------------------------------>

            //moves robot to correct parking position
            if (detect1.getParkPosition() == ContourMultiScoreLeft.ParkingPosition.LEFT) {
                // go forward
                drive.goToPosition(0.62, 0.6, 0.62, 0.6, 880, "go forwards");
                // strafe left
                drive.goToPosition(-0.6, 0.6, 0.6, -0.6, 501, "strafe left");

            } else if (detect1.getParkPosition() == ContourMultiScoreLeft.ParkingPosition.CENTER) {
                // go forward
                drive.goToPosition(0.62, 0.6, 0.62, 0.6, 150, "go forwards");
                // strafe left
                drive.goToPosition(-0.6, 0.6, 0.6, -0.6, 501, "strafe left");
            } else {
                // go backwards
                drive.goToPosition(-0.6, -0.6, -0.6, -0.6, 125, "go backwards");
                // strafe left
                drive.goToPosition(-0.6, 0.6, 0.6, -0.6, 501, "strafe left");


            }
            score.goToPosition(0, 0.85);


        }

    }

    public void scoreConePreload(int fl, int fr, int bl, int br) {
        /*
        // move arm medium
        lifting = true;
        score.goToPositionPID(constants.getHeightMed()+30)+10, 0.85);
        lifting = false;
        */

        //begin thread for maintaining height of slides

        onTimeout = System.currentTimeMillis();
        //3700 - 3800
        drive.goToPosition(0.12, 0.12, 0.12, 0.12);
        sleep(100);
        // old: 7900     (9100 new)  (6800 new)  7400

        while (detect1.getBoundArea() <= 5700.0 || detect1.getBoundArea() >= 8400) { //7200
            if (detect1.getBoundArea() >= 5700.0 && detect1.getBoundArea() <= 8400 && detect1.getDistance() <= 5.5/*|| detect1.getcX() <= 18*/) {
                drive.goToPosition(0, 0, 0, 0);
                break;
            } else if ((System.currentTimeMillis() - onTimeout) / 1000 > 3) {
                break;
            }

        }




        sleep(200);
        score.setClawPosition(constants.getClawOpenPos());

        //move back from pole
        drive.goToPosition(-0.5, -0.5, -0.5, -0.5, fl, "move back from pole");

        //lower cone ontto pole
        liftTurn.set(true);




    }

    public void scoreConeMulti(int fl, int fr, int bl, int br) {
        /*
        // move arm medium
        lifting = true;
        score.goToPositionPID(constants.getHeightMed()+30)+10, 0.85);
        lifting = false;
        */

        //begin thread for maintaining height of slides

        onTimeout = System.currentTimeMillis();
        //3700 - 3800
        drive.goToPosition(0.1, 0.1, 0.1, 0.1);
        // old: 7900     (9100 new)  (6800 new)  7400

        while (detect1.getBoundArea() <= 5000.0 || detect1.getBoundArea() >= 8400) { //7200
            if (detect1.getBoundArea() >= 5000.0 && detect1.getBoundArea() <= 8400 && detect1.getDistance() <= 5.5/*|| detect1.getcX() <= 18*/) {
                drive.goToPosition(0, 0, 0, 0);
                break;
            } else if ((System.currentTimeMillis() - onTimeout) / 1000 > 3) {
                break;
            }

        }




        sleep(300);
        score.setClawPosition(constants.getClawOpenPos());

        //move back from pole
        drive.goToPosition(-0.5, -0.5, -0.5, -0.5, fl, "move back from pole");

        //lower cone ontto pole
        liftTurn.set(true);




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
            } else if ((System.currentTimeMillis() - onTimeout) / 1000 > 3) {
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

    public void safeParking(boolean grabbingCone, boolean findingTape, boolean scoringPole, boolean strafing) {
        drive.goToPosition(0 , 0, 0, 0);

        if (grabbingCone) {
            // turn correction
            if (Math.abs(drive.getAngularOrientation() - Math.PI / 2) > 0.1) {
                drive.turn(Math.PI / 2);
            }

            // PARK ------------------------------------------------------>

            //moves robot to correct parking position
            if (detect1.getParkPosition() == ContourMultiScoreLeft.ParkingPosition.LEFT) {
                // go backwards a bit
                drive.goToPosition(-0.5, -0.52, -0.5, -0.52, 70, "go backwards a bit");
                // strafe left
                drive.goToPosition(-0.6, 0.6, 0.6, -0.6, 501, "strafe left");

            } else if (detect1.getParkPosition() == ContourMultiScoreLeft.ParkingPosition.CENTER) {
                // go backwards
                drive.goToPosition(-0.5, -0.52, -0.5, -0.52, 300, "go backwards a bit");
                // strafe left
                drive.goToPosition(-0.6, 0.6, 0.6, -0.6, 501, "strafe left");
            } else {
                // go backwards
                drive.goToPosition(-0.5, -0.52, -0.5, -0.52, 500, "go backwards");
                // strafe left
                drive.goToPosition(-0.6, 0.6, 0.6, -0.6, 501, "strafe left");


            }
            score.goToPosition(0, 0.85);
        } else if (findingTape) {
            // turn correction
            if (Math.abs(drive.getAngularOrientation() - Math.PI / 2) > 0.1) {
                drive.turn(Math.PI / 2);
            }
            // go backwards
            drive.goToPosition(-0.5, -0.52, -0.5, -0.52, 50, "go backwards");

            //strafe left
            drive.goToPosition(-0.5, 0.5, 0.5, -0.5, 600, "strafe left");


            // PARK ------------------------------------------------------>

            //moves robot to correct parking position
            if (detect1.getParkPosition() == ContourMultiScoreLeft.ParkingPosition.LEFT) {
                // go backwards a bit
                drive.goToPosition(0.52, 0.5, 0.52, 0.5, 30, "go forwards a bit");
                // strafe left
                drive.goToPosition(-0.6, 0.6, 0.6, -0.6, 501, "strafe left");

            } else if (detect1.getParkPosition() == ContourMultiScoreLeft.ParkingPosition.CENTER) {
                // go backwards
                drive.goToPosition(-0.5, -0.52, -0.5, -0.52, 300, "go backwards a bit");
                // strafe left
                drive.goToPosition(-0.6, 0.6, 0.6, -0.6, 501, "strafe left");
            } else {
                // go backwards
                drive.goToPosition(-0.5, -0.52, -0.5, -0.52, 500, "go backwards");
                // strafe left
                drive.goToPosition(-0.6, 0.6, 0.6, -0.6, 501, "strafe left");


            }
            score.goToPosition(0, 0.85);
        } else if (scoringPole) {
            // move backwards (away from pole)
            drive.goToPosition(-0.5, -0.52, -0.5, -0.52, 100, "go backwards");


            //moves robot to correct parking position
            drive.absTurnPID(Math.PI / 2);


            //}
            // PARK ------------------------------------------------------>

            //moves robot to correct parking position
            if (detect1.getParkPosition() == ContourMultiScoreLeft.ParkingPosition.LEFT) {
                // go forward
                drive.goToPosition(0.62, 0.6, 0.62, 0.6, 880, "go forwards");
                // strafe left
                drive.goToPosition(-0.6, 0.6, 0.6, -0.6, 501, "strafe left");

            } else if (detect1.getParkPosition() == ContourMultiScoreLeft.ParkingPosition.CENTER) {
                // go forward
                drive.goToPosition(0.62, 0.6, 0.62, 0.6, 150, "go forwards");
                // strafe left
                drive.goToPosition(-0.6, 0.6, 0.6, -0.6, 501, "strafe left");
            } else {
                // go backwards
                drive.goToPosition(-0.6, -0.6, -0.6, -0.6, 125, "go backwards");
                // strafe left
                drive.goToPosition(-0.6, 0.6, 0.6, -0.6, 501, "strafe left");


            }




        } else if (strafing) {

            //moves robot to correct parking position
            drive.absTurnPID(Math.PI / 2);


            //}
            // PARK ------------------------------------------------------>

            //moves robot to correct parking position
            if (detect1.getParkPosition() == ContourMultiScoreLeft.ParkingPosition.LEFT) {
                // go forward
                drive.goToPosition(0.62, 0.6, 0.62, 0.6, 880, "go forwards");
                // strafe left
                drive.goToPosition(-0.6, 0.6, 0.6, -0.6, 501, "strafe left");

            } else if (detect1.getParkPosition() == ContourMultiScoreLeft.ParkingPosition.CENTER) {
                // go forward
                drive.goToPosition(0.62, 0.6, 0.62, 0.6, 150, "go forwards");
                // strafe left
                drive.goToPosition(-0.6, 0.6, 0.6, -0.6, 501, "strafe left");
            } else {
                // go backwards
                drive.goToPosition(-0.6, -0.6, -0.6, -0.6, 60, "go backwards");
                // strafe left
                drive.goToPosition(-0.6, 0.6, 0.6, -0.6, 501, "strafe left");


            }
        }

        safePark.set(false);
        this.strafing = false;
        this.scoringPole = false;
        this.findingTape = false;
        this.grabbingCone = false;

        score.goToPosition(0, 0.8);
        sleep(1000000);
    }

    public void useColorSensor() {
        drive.findTape("blueleft", true);
        //makes sure actually turned 90 degrees
        if (Math.abs(drive.getAngularOrientation() - Math.PI / 2) > 0.15) {
            drive.turn(Math.PI / 2);
        }

        score.goToPosition(constants.getStackHeight(), 0.7);

        score.grabConeAuto();


    }
}
