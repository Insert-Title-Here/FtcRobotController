package org.firstinspires.ftc.teamcode.State.MultiAuto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.State.Auto.ContourMultiScoreLeft1;
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
public class MultiBlueLeftHIGH1 extends LinearOpMode {

    // instantiating
    Thread liftThread;
    MecanumDrive drive;
    ContourMultiScoreLeft1 detect1;
    ScoringSystem score;
    Constants constants;
    OpenCvWebcam webcam;
    AtomicBoolean liftTurn, liftCone, safePark;

    // Running variable storing the pipeline to use
    private static volatile Constants.Pipeline normalization;

    // ftc dashboard values + properCX
    private double properCX = 167; //67  165
    private double properCXHigh = 170; //67
    private double properCXCone = 165;

    private double properCXLow = 180; //160  163

    public static boolean togg = false;
    public static int pos = 0;
    public static int positive_negative = 1;
    public static int turnDenom = 4;
    public static int toggle = 1;
    public static int inner_toggle = 0;
    private boolean toggleIFS = true;
    volatile boolean earlyPark = false;
    volatile boolean lifting = false;
    volatile boolean grabbingCone = false;
    volatile boolean findingTape = false;
    volatile boolean scoringPole = false;
    volatile boolean strafing = false;
    volatile long fullTime;

    long time;
    long onTimeout;

    private int sum = -13;


    @Override
    public void runOpMode() throws InterruptedException {
        // value initializing
        detect1 = new ContourMultiScoreLeft1(telemetry);
        drive = new MecanumDrive(hardwareMap, telemetry);
        //sets Auto driving to break instead of float
        
        score = new ScoringSystem(hardwareMap, telemetry);
        constants = new Constants();
        liftTurn = new AtomicBoolean();
        liftCone = new AtomicBoolean();
        safePark = new AtomicBoolean();
        liftCone.set(false);
        liftTurn.set(false);
        safePark.set(false);

        detect1.setIsBlue(true);

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
                        stackPos(pos);


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

                            if ((System.currentTimeMillis() - time) / 1000 > 2.5 || (System.currentTimeMillis() - fullTime) / 1000 < 2.5) {
                                safeParking(true, false, false, false);

                            }
                        }
                        while (findingTape) {
                            if ((System.currentTimeMillis() - time) / 1000 > 2.5 || (System.currentTimeMillis() - fullTime) / 1000 < 2.5) {
                                safeParking(false, true, false, false);

                            }
                        }
                        while (scoringPole) {
                            if ((System.currentTimeMillis() - time) / 1000 > 3 || (System.currentTimeMillis() - fullTime) / 1000 < 2.5) {
                                safeParking(false, false, true, false);

                            }
                        }
                        while (strafing) {
                            if ((System.currentTimeMillis() - time) / 1000 > 1.5 || (System.currentTimeMillis() - fullTime) / 1000 < 2.5) {
                                safeParking(false, false, false, true);

                            }
                        }
                    }

//
                    telemetry.addData("imu", drive.currentAngle());
                    telemetry.addData("power", drive.getBlPower());
                    telemetry.update();
                }

            }
        };

        //properCX = detect1.getBoundWidth();
        // code to turn servo of cam
        //score.setCamPosition(constants.getSleeveCamPos());
        //detect1.park = true;


        normalization = Constants.Pipeline.PARK;


        // ftc dashboard setup
        FtcDashboard.getInstance().startCameraStream(webcam, 0);
        telemetry.addData("Status", "Initialized");

        telemetry.update();

        //close claw
        score.setClawPosition(constants.getClawClosePos()); //TODO

        // turn servo of cam forward for sleeves
        score.setCamPosition(constants.getSleeveCamPos());


        waitForStart();
        fullTime = System.currentTimeMillis();
        normalization = Constants.Pipeline.POLE;
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

            //lift claw a little bit
            score.goToPosition(50, 0.8);
            sleep(100);
            // go forward next to pole
            drive.goToPosition(0.74, 0.7, 0.74, 0.7, 1380, "go forward next to pole");
            // turn to right  45 degrees to high pole
            drive.absTurnDriftPID(-Math.PI / 4);
//            // go to pole a bit
            drive.goToPosition(0.62, 0.6, 0.62, 0.6, 50, "go forward some to pole");
            // move arm high
            lifting = true;
            score.goToPositionPID(constants.getHeightHigh(), 0.85);
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


            pos = 5;
            scoreConePreload(100, 0, 0, 0);
            drive.goToPosition(-0.6, -0.6, -0.6, -0.6, 60, "back up a little");
            // turn to left
//            drive.turn90(Math.PI / 4 + Math.PI / 1.9);
            drive.absTurnPID(Math.PI / 2.15);

//            //makes sure actually turned 90 degrees
//            if (Math.abs(drive.getAngularOrientation() - Math.PI / 2) > 0.05) {
//                drive.absTurnPID(Math.PI / 2);
//            }

            score.setCamPosition(constants.getStrafeConeCamPos());

            normalization = Constants.Pipeline.CONE;

            score.setClawPosition(constants.getClawOpenPos());
            //go forward a bit
            //drive.goToPosition(0.52, 0.5, 0.52, 0.5, 220, "forward");

            //drive.goToPosition(-0.5, 0.5, 0.5, -0.5, 110, "to the left a bit");


            // +'s ---------------------------------------->

            pos = 4;



            /*
            //makes sure actually turned 90 degrees
            if (Math.abs(drive.getAngularOrientation() - Math.PI / 2) > 0.05) {
                drive.turn(Math.PI / 2);
            }

             */

            safePark.set(true);
            findingTape = true;
            findCone();

            safePark.set(false);
            findingTape = false;

            score.setCamPosition(constants.getSleeveCamPos());

            normalization = Constants.Pipeline.POLE;
            /*
            //makes sure actually turned 90 degrees
            if (Math.abs(drive.getAngularOrientation() - Math.PI / 2) > 0.05) {
                drive.turn(Math.PI / 2);
            }

             */
            drive.goToPosition(0.52, 0.5, 0.52, 0.5, 250, "forward a bit");
            safePark.set(true);
            grabbingCone = true;
            score.grabConeAuto();
            safePark.set(false);
            grabbingCone = false;

            //strafe to the left a bit
            //drive.goToPosition(-0.6, 0.6, 0.6, -0.6, 20, "strafe to left a bit");

            //makes sure actually turned 90 degrees
            if (Math.abs(drive.getAngularOrientation() - Math.PI / 1.9) > 0.05) {
                drive.absTurnPID(Math.PI / 1.9);
            }
            //drive.goToPosition(-0.5, 0.5, 0.5, -0.5, 35, "go to left a bit");

            sleep(50);
            drive.goToPosition(-0.7, -0.7, -0.7, -0.7, 450, "go backwards");

            // turn to right  45 degrees to high pole
            drive.absTurnDriftPID(-Math.PI / 3.7);
//            // go to pole a bit
            drive.goToPosition(0.62, 0.6, 0.62, 0.6, 60, "go forward some to pole");
            // move arm high
            lifting = true;
            score.goToPositionPID(constants.getHeightHigh(), 0.85);
            lifting = false;
            sleep(50);

            right = true;
            left = true;


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


            sum = 15;
            scoreConePreload(100, 0, 0, 0);
            // turn to left
//            drive.turn90(Math.PI / 4 + Math.PI / 1.9);
            drive.absTurnPID(Math.PI / 2.22);

            score.setCamPosition(constants.getStrafeConeCamPos());

            normalization = Constants.Pipeline.CONE;

            score.setClawPosition(constants.getClawOpenPos());


            //go forward a bit
            //drive.goToPosition(0.52, 0.5, 0.52, 0.5, 220, "forward");

            //drive.goToPosition(-0.5, 0.5, 0.5, -0.5, 110, "to the left a bit");


            // +2s ---------------------------------------->

            pos = 3;



            /*
            //makes sure actually turned 90 degrees
            if (Math.abs(drive.getAngularOrientation() - Math.PI / 2) > 0.05) {
                drive.turn(Math.PI / 2);
            }

             */
            drive.goToPosition(0.52, 0.5, 0.52, 0.5, 40, "forward a bit");
            safePark.set(true);
            findingTape = true;
            findCone();

            safePark.set(false);
            findingTape = false;

            score.setCamPosition(constants.getSleeveCamPos() + 0.02);

            normalization = Constants.Pipeline.POLE;
            /*
            //makes sure actually turned 90 degrees
            if (Math.abs(drive.getAngularOrientation() - Math.PI / 2) > 0.05) {
                drive.turn(Math.PI / 2);
            }

             */
            drive.goToPosition(0.53, 0.5, 0.53, 0.5, 250, "forward a bit");
            safePark.set(true);
            grabbingCone = true;
            score.grabConeAuto();
            safePark.set(false);
            grabbingCone = false;

            //strafe to the left a bit
            //drive.goToPosition(-0.6, 0.6, 0.6, -0.6, 20, "strafe to left a bit");

            //makes sure actually turned 90 degrees
            if (Math.abs(drive.getAngularOrientation() - Math.PI / 1.9) > 0.05) {
                drive.absTurnPID(Math.PI / 1.9);
            }
            //drive.goToPosition(-0.5, 0.5, 0.5, -0.5, 35, "go to left a bit");

            sleep(50);
            drive.goToPosition(-0.7, -0.7, -0.7, -0.7, 450, "go backwards");

            // turn to right  45 degrees to high pole
            drive.absTurnDriftPID(-Math.PI / 3.7);
//            // go to pole a bit
            drive.goToPosition(0.62, 0.6, 0.62, 0.6, 60, "go forward some to pole");
            // move arm high
            lifting = true;
            score.goToPositionPID(constants.getHeightHigh(), 0.85);
            lifting = false;
            sleep(50);

            right = true;
            left = true;


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


//            pos = 2;
            scoreConePreload(0, 0, 0, 0);
            drive.goToPosition(-0.6, -0.6, -0.6, -0.6, 80, "back up a little");
            // turn to left
//            drive.turn90(Math.PI / 4 + Math.PI / 1.9);
            drive.absTurnPID(Math.PI / 2.21);

//            //makes sure actually turned 90 degrees
//            if (Math.abs(drive.getAngularOrientation() - Math.PI / 2) > 0.05) {
//                drive.absTurnPID(Math.PI / 2);
//            }

            score.setCamPosition(constants.getStrafeConeCamPos());

            normalization = Constants.Pipeline.CONE;

            score.setClawPosition(constants.getClawOpenPos());

//            if (!earlyPark) {
//                // +3s ---------------------------------------->
//
//                sum += constants.getStackIntervalHeight() + 10;
//
//
//
//                /*
//                //makes sure actually turned 90 degrees
//                if (Math.abs(drive.getAngularOrientation() - Math.PI / 2) > 0.05) {
//                    drive.turn(Math.PI / 2);
//                }
//
//                 */
//                drive.goToPosition(0.52, 0.5, 0.52, 0.5, 40, "forward a bit");
//
//                safePark.set(true);
//                findingTape = true;
//                findCone();
//
//                safePark.set(false);
//                findingTape = false;
//
//                score.setCamPosition(constants.getSleeveCamPos() + 0.03);
//
//                normalization = Constants.Pipeline.POLE;
//                /*
//                //makes sure actually turned 90 degrees
//                if (Math.abs(drive.getAngularOrientation() - Math.PI / 2) > 0.05) {
//                    drive.turn(Math.PI / 2);
//                }
//
//                 */
//                drive.goToPosition(0.55, 0.5, 0.55, 0.5, 250, "forward a bit");
//                safePark.set(true);
//                grabbingCone = true;
//                score.grabConeAuto();
//                safePark.set(false);
//                grabbingCone = false;
//
//                //strafe to the left a bit
//                //drive.goToPosition(-0.6, 0.6, 0.6, -0.6, 20, "strafe to left a bit");
//
//                //makes sure actually turned 90 degrees
//                if (Math.abs(drive.getAngularOrientation() - Math.PI / 1.9) > 0.05) {
//                    drive.absTurnPID(Math.PI / 1.9);
//                }
//                //drive.goToPosition(-0.5, 0.5, 0.5, -0.5, 35, "go to left a bit");
//
//                sleep(50);
//                drive.goToPosition(-0.7, -0.7, -0.7, -0.7, 450, "go backwards");
//
//                // turn to right  45 degrees to high pole
//                drive.absTurnDriftPID(-Math.PI / 3.7);
//                // go to pole a bit
//                drive.goToPosition(0.62, 0.6, 0.62, 0.6, 60, "go forward some to pole");
//                // move arm high
//                lifting = true;
//                score.goToPositionPID(constants.getHeightHigh(), 0.85);
//                lifting = false;
//                sleep(50);
//
//                right = true;
//                left = true;
//
//
//                // camera position correction
//                while (detect1.getcX() < properCXHigh - 5 || detect1.getcX() > properCXHigh + 5) {
//                    telemetry.addData("success", "sucess");
//                    telemetry.update();
//                    if (detect1.getcX() < properCXHigh - 5 && right) {
//                        // strafe to the right
//                        //                drive.goToPosition(0.15, -0.15, -0.15, 0.15);
//                        drive.goToPosition(-0.265, 0.265, 0.265, -0.265);
//
//
//                        telemetry.addData("success 1", "sucess");
//                        telemetry.update();
//                        right = false;
//                    }
//                    if (detect1.getcX() > properCXHigh + 5 && left) {
//                        // strafe to the left (change fr and bl)
//                        drive.goToPosition(0.265, -0.265, -0.265, 0.265);
//
//
//                        //                drive.goToPosition(-0.15, 0.15, 0.15, -0.15);
//
//                        telemetry.addData("success 2", "sucess");
//                        telemetry.update();
//                        left = false;
//                    }
//                    //            if (detect1.getcX() >= properCX - 5 && detect1.getcX() <= properCX + 5) {
//                    //                drive.goToPosition(0, 0, 0, 0);
//                    //            }
//                }
//
//
//                drive.goToPosition(0, 0, 0, 0);
//
//
//                scoreConePreload(100, 0, 0, 0);
//                // turn to left
//    //            drive.turn90(Math.PI / 4 + Math.PI / 1.9);
//                drive.absTurnPID(Math.PI / 2.23);
//
//    //            //makes sure actually turned 90 degrees
//    //            if (Math.abs(drive.getAngularOrientation() - Math.PI / 2) > 0.05) {
//    //                drive.absTurnPID(Math.PI / 2);
//    //            }
//
//                score.setCamPosition(constants.getStrafeConeCamPos());
//
//                normalization = Constants.Pipeline.CONE;
//
//                score.setClawPosition(constants.getClawOpenPos());
//
//
//            }



            // PARK ------------------------------------------------------>

            //moves robot to correct parking position
            if (detect1.getParkPosition() == ContourMultiScoreLeft1.ParkingPosition.LEFT) {
                // go forward
                drive.goToPosition(0.74, 0.7, 0.74, 0.7, 650, "go forwards");
                sleep(100);
                // strafe left
                drive.goToPosition(-0.7, 0.7, 0.7, -0.7, 100, "strafe left");

            } else if (detect1.getParkPosition() == ContourMultiScoreLeft1.ParkingPosition.CENTER) {
                // go forward
                drive.goToPosition(0.74, 0.7, 0.74, 0.7, 80, "go forwards");
                sleep(100);

                // strafe left
                drive.goToPosition(-0.7, 0.7, 0.7, -0.7, 200, "strafe left");
            } else {
                // go backwards
                drive.goToPosition(-0.7, -0.7, -0.7, -0.7, 390, "go backwards");
                sleep(100);

                // strafe left
                drive.goToPosition(-0.7, 0.7, 0.7, -0.7, 200, "strafe left");


            }
            score.goToPosition(0, 0.85);




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

            while (opModeIsActive() ) {

                if (togg) {
                    stackPos(pos);
                }
            }

        }else {
//lift claw a little bit
            score.goToPosition(50, 0.8);
            sleep(100);
            // go forward next to pole
            drive.goToPosition(0.74, 0.7, 0.74, 0.7, 1300, "go forward next to pole");
            // turn to right  45 degrees to high pole
            drive.absTurnDriftPID(-Math.PI / 4);
//            // go to pole a bit
            drive.goToPosition(0.62, 0.6, 0.62, 0.6, 50, "go forward some to pole");
            // move arm high
            lifting = true;
            score.goToPositionPID(constants.getHeightHigh(), 0.85);
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




            sum = 15;
            scoreConePreload(100, 0, 0, 0);
            drive.goToPosition(-0.6, -0.6, -0.6, -0.6, 80, "back up a little");
            // turn to left
//            drive.turn90(Math.PI / 4 + Math.PI / 1.9);
            drive.absTurnPID(Math.PI / 2);
            score.setCamPosition(constants.getStrafeConeCamPos());

            normalization = Constants.Pipeline.CONE;

            score.setClawPosition(constants.getClawOpenPos());
            //go forward a bit
            //drive.goToPosition(0.52, 0.5, 0.52, 0.5, 220, "forward");

            //drive.goToPosition(-0.5, 0.5, 0.5, -0.5, 110, "to the left a bit");


            // +'s ---------------------------------------->

            sum += constants.getStackIntervalHeight();



            /*
            //makes sure actually turned 90 degrees
            if (Math.abs(drive.getAngularOrientation() - Math.PI / 2) > 0.05) {
                drive.turn(Math.PI / 2);
            }

             */

            safePark.set(true);
            findingTape = true;
            findCone();

            safePark.set(false);
            findingTape = false;

            score.setCamPosition(constants.getSleeveCamPos());

            normalization = Constants.Pipeline.POLE;
            /*
            //makes sure actually turned 90 degrees
            if (Math.abs(drive.getAngularOrientation() - Math.PI / 2) > 0.05) {
                drive.turn(Math.PI / 2);
            }

             */
            drive.goToPosition(0.52, 0.5, 0.52, 0.5, 250, "forward a bit");
            safePark.set(true);
            grabbingCone = true;
            score.grabConeAuto();
            safePark.set(false);
            grabbingCone = false;

            //strafe to the left a bit
            //drive.goToPosition(-0.6, 0.6, 0.6, -0.6, 20, "strafe to left a bit");

            //makes sure actually turned 90 degrees
            if (Math.abs(drive.getAngularOrientation() - Math.PI / 1.9) > 0.05) {
                drive.absTurnPID(Math.PI / 1.9);
            }
            //drive.goToPosition(-0.5, 0.5, 0.5, -0.5, 35, "go to left a bit");

            sleep(50);
            drive.goToPosition(-0.7, -0.7, -0.7, -0.7, 820, "go backwards");

            drive.absTurnDriftPID(0);

            lifting = true;
            score.goToPositionPID(constants.getHeightHigh(), 0.85);
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
            //checking (git)


            safePark.set(true);
            scoringPole = true;
            //scoreConeMulti(30, 150, 100, 100);
            drive.goToPosition(0.3, 0.3, 0.3, 0.3, 100, "go forward to pole");
            sleep(300);
            score.setClawPosition(constants.getClawOpenPos());

            //move back from pole
            drive.goToPosition(-0.6, -0.6, -0.6, -0.6, 30, "move back from pole");

            //lower cone ontto pole
            liftTurn.set(true);

            safePark.set(false);
            scoringPole = false;


            drive.absTurnPID(Math.PI / 2);

            //makes sure actually turned 90 degrees
            if (Math.abs(drive.getAngularOrientation() - Math.PI / 1.9) > 0.05) {
                drive.turn(Math.PI / 2);
            }

            drive.goToPosition(0.74, 0.7, 0.74, 0.7, 700, "go forwards");


            normalization = Constants.Pipeline.CONE;

            score.setCamPosition(constants.getStrafeConeCamPos());
            /*
            //makes sure actually turned 90 degrees
            if (Math.abs(drive.getAngularOrientation() - Math.PI / 1.9) > 0.05) {
                drive.turn(Math.PI / 1.9);
            }

             */

            safePark.set(true);
            findingTape = true;
            findCone();

            safePark.set(false);
            findingTape = false;
            /*
            //makes sure actually turned 90 degrees
            if (Math.abs(drive.getAngularOrientation() - Math.PI / 1.9) > 0.05) {
                drive.turn(Math.PI / 1.9);
            }

             */

            score.setCamPosition(constants.getSleeveCamPos());
            normalization = Constants.Pipeline.POLE;

            // + 2 ---------------------------------------
            sum += constants.getStackIntervalHeight();




//            //makes sure actually turned 90 degrees
//            if (Math.abs(drive.getAngularOrientation() - Math.PI / 2) > 0.05) {
//                drive.turn(Math.PI / 2);
//            }



//            //makes sure actually turned 90 degrees
//            if (Math.abs(drive.getAngularOrientation() - Math.PI / 2) > 0.05) {
//                drive.turn(Math.PI / 2);
//            }

            safePark.set(true);
            grabbingCone = true;
            score.grabConeAuto();
            safePark.set(false);
            grabbingCone = false;


            //strafe to the left a bit
            //drive.goToPosition(-0.6, 0.6, 0.6, -0.6, 20, "strafe to left a bit");

            //makes sure actually turned 90 degrees
            if (Math.abs(drive.getAngularOrientation() - Math.PI / 1.9) > 0.05) {
                drive.absTurnPID(Math.PI / 1.97);
            }

            //drive.goToPosition(-0.5, 0.5, 0.5, -0.5, 35, "go to left a bit");

            sleep(50);
            drive.goToPosition(-0.7, -0.7, -0.7, -0.7, 780, "go backwards");

            drive.absTurnDriftPID(0);

            lifting = true;
            score.goToPositionPID(constants.getHeightHigh(), 0.85);
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


            /*
            drive.goToPosition(0.3, 0.3, 0.3, 0.3, 80, "go forward to pole");
            sleep(300);
            score.setClawPosition(constants.getClawOpenPos());

            //move back from pole
            drive.goToPosition(-0.6, -0.6, -0.6, -0.6, 30, "move back from pole");



            //lower cone ontto pole
            liftTurn.set(true);
*/
            safePark.set(false);
            scoringPole = false;


            drive.absTurnPID(Math.PI / 2);
            //makes sure actually turned 90 degrees
            if (Math.abs(drive.getAngularOrientation() - Math.PI / 1.9) > 0.05) {
                drive.absTurnPID(Math.PI / 2);
            }

            drive.goToPosition(0.73, 0.7, 0.73, 0.7, 600, "go forwards");


            normalization = Constants.Pipeline.CONE;

            score.setCamPosition(constants.getStrafeConeCamPos());
            /*
            //makes sure actually turned 90 degrees
            if (Math.abs(drive.getAngularOrientation() - Math.PI / 1.9) > 0.05) {
                drive.turn(Math.PI / 1.9);
            }

             */

            safePark.set(true);
            findingTape = true;
            findCone();

            safePark.set(false);
            findingTape = false;
            /*
            //makes sure actually turned 90 degrees
            if (Math.abs(drive.getAngularOrientation() - Math.PI / 1.88) > 0.05) {
                drive.turn(Math.PI / 1.88);
            }

             */

            score.setCamPosition(constants.getSleeveCamPos());
            normalization = Constants.Pipeline.POLE;


            // +3 ------------------------------------->>>>


            sum += constants.getStackIntervalHeight();




//            //makes sure actually turned 90 degrees
//            if (Math.abs(drive.getAngularOrientation() - Math.PI / 2) > 0.05) {
//                drive.turn(Math.PI / 2);
//            }



//            //makes sure actually turned 90 degrees
//            if (Math.abs(drive.getAngularOrientation() - Math.PI / 2) > 0.05) {
//                drive.turn(Math.PI / 2);
//            }

            safePark.set(true);
            grabbingCone = true;
            score.grabConeAuto();
            safePark.set(false);
            grabbingCone = false;


            //strafe to the left a bit
            // drive.goToPosition(-0.6, 0.6, 0.6, -0.6, 20, "strafe to left a bit");

            //makes sure actually turned 90 degrees
            if (Math.abs(drive.getAngularOrientation() - Math.PI / 1.9) > 0.05) {
                drive.absTurnPID(Math.PI / 2);
            }

            //drive.goToPosition(-0.5, 0.5, 0.5, -0.5, 35, "go to left a bit");

            sleep(50);
            drive.goToPosition(-0.7, -0.7, -0.7, -0.7, 820, "go backwards");

            drive.absTurnDriftPID(0);

            lifting = true;
            score.goToPositionPID(constants.getHeightHigh(), 0.85);
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

            /*
            drive.goToPosition(0.3, 0.3, 0.3, 0.3, 80, "go forward to pole");
            sleep(300);
            score.setClawPosition(constants.getClawOpenPos());

            //move back from pole
            drive.goToPosition(-0.6, -0.6, -0.6, -0.6, 30, "move back from pole");

            //lower cone ontto pole
            liftTurn.set(true);
            */

            safePark.set(false);
            scoringPole = false;


            drive.absTurnPID(Math.PI / 2);
            //makes sure actually turned 90 degrees
            if (Math.abs(drive.getAngularOrientation() - Math.PI / 1.9) > 0.05) {
                drive.absTurnPID(Math.PI / 2);
            }


            // PARK ------------------------------------------------------>

            //moves robot to correct parking position
            if (detect1.getParkPosition() == ContourMultiScoreLeft1.ParkingPosition.LEFT) {
                // go forward
                drive.goToPosition(0.62, 0.6, 0.62, 0.6, 700, "go forwards");
                // strafe left
                drive.goToPosition(-0.6, 0.6, 0.6, -0.6, 501, "strafe left");

            } else if (detect1.getParkPosition() == ContourMultiScoreLeft1.ParkingPosition.CENTER) {
                // go forward
                drive.goToPosition(0.62, 0.6, 0.62, 0.6, 150, "go forwards");
                // strafe left
                drive.goToPosition(-0.6, 0.6, 0.6, -0.6, 501, "strafe left");
            } else {
                // go backwards
                drive.goToPosition(-0.6, -0.6, -0.6, -0.6, 140, "go backwards");
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
        drive.goToPosition(0.14, 0.14, 0.14, 0.14);
        // old: 7900     (9100 new)  (6800 new)  states --> 6400

        while (detect1.getBoundArea() <= 6500.0 || detect1.getBoundArea() >= 8400) { //7200
            if (detect1.getBoundArea() >= 4900.0 && detect1.getBoundArea() <= 8400 && detect1.getDistance() <= 5.5/*|| detect1.getcX() <= 18*/) {
                drive.goToPosition(0, 0, 0, 0);
                break;
            } else if ((System.currentTimeMillis() - onTimeout) / 1000 > 3) {
                break;
            }

        }




        sleep(100);
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
        drive.goToPosition(0.13, 0.13, 0.13, 0.13);
        // old: 7900     (9100 new)  (6800 new)  7400

        while (detect1.getBoundArea() <= 6000.0 || detect1.getBoundArea() >= 8400) { //7200
            if (detect1.getBoundArea() >= 6000.0 && detect1.getBoundArea() <= 8400 /*&& detect1.getDistance() <= 5.5*/ /*|| detect1.getcX() <= 18*/) {
                drive.goToPosition(0, 0, 0, 0);
                break;
            } else if ((System.currentTimeMillis() - onTimeout) / 1000 > 3) {
                break;
            }

        }




        sleep(300);
        score.setClawPosition(constants.getClawOpenPos());
        liftTurn.set(true);
        //move back from pole
        drive.goToPosition(-0.5, -0.5, -0.5, -0.5, fl, "move back from pole");

        //lower cone ontto pole




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

    public void findCone() {
        if (opModeIsActive()) {

            boolean left = true;
            boolean right = true;

            // camera position correction
            while (detect1.getcX() < properCXCone - 5 || detect1.getcX() > properCXCone + 5) {
                telemetry.addData("success", "sucess");
                telemetry.update();
                if (detect1.getcX() < properCXCone - 5 && right) {
                    // strafe to the right
                    //                drive.goToPosition(0.15, -0.15, -0.15, 0.15);
                    drive.goToPosition(-0.26, 0.26, 0.26, -0.26);


                    telemetry.addData("success 1", "sucess");
                    telemetry.update();
                    right = false;
                    left = true;
                }
                if (detect1.getcX() > properCXCone + 5 && left) {
                    // strafe to the left (change fr and bl)
                    drive.goToPosition(0.26, -0.26, -0.26, 0.26);


                    //                drive.goToPosition(-0.15, 0.15, 0.15, -0.15);

                    telemetry.addData("success 2", "sucess");
                    telemetry.update();
                    left = false;
                    right = true;
                }
                //            if (detect1.getcX() >= properCX - 5 && detect1.getcX() <= properCX + 5) {
                //                drive.goToPosition(0, 0, 0, 0);
                //            }
            }
            drive.goToPosition(0, 0, 0, 0);
        }
    }

    public void safeParking(boolean grabbingCone, boolean findingTape, boolean scoringPole, boolean strafing) {
        boolean  running = true;
        if (opModeIsActive()) {

            drive.goToPosition(0, 0, 0, 0);

            if (grabbingCone && running) {
                // move back a bit first
                drive.goToPosition(-0.5, -0.52, -0.5, -0.52, 60, "go backwards a bit");

                // turn correction
                if (Math.abs(drive.getAngularOrientation() - Math.PI / 2) > 0.1) {
                    drive.turn(Math.PI / 2);
                }

                // PARK ------------------------------------------------------>

                //moves robot to correct parking position
                if (detect1.getParkPosition() == ContourMultiScoreLeft1.ParkingPosition.LEFT) {
                    // go backwards a bit
                    drive.goToPosition(-0.5, -0.52, -0.5, -0.52, 70, "go backwards a bit");
                    // strafe left
                    drive.goToPosition(-0.6, 0.6, 0.6, -0.6, 501, "strafe left");

                } else if (detect1.getParkPosition() == ContourMultiScoreLeft1.ParkingPosition.CENTER) {
                    // go backwards
                    drive.goToPosition(-0.5, -0.52, -0.5, -0.52, 300, "go backwards a bit");
                    // strafe left
                    drive.goToPosition(-0.6, 0.6, 0.6, -0.6, 501, "strafe left");
                } else {
                    // go backwards
                    drive.goToPosition(-0.5, -0.52, -0.5, -0.52, 1400, "go backwards");
                    // strafe left
                    drive.goToPosition(-0.6, 0.6, 0.6, -0.6, 501, "strafe left");


                }
                score.goToPosition(0, 0.85);
                running = false;
            } else if (findingTape && running) {
                // go backwards
                drive.goToPosition(-0.5, -0.52, -0.5, -0.52, 50, "go backwards");

                // turn correction
                if (Math.abs(drive.getAngularOrientation() - Math.PI / 2) > 0.1) {
                    drive.turn(Math.PI / 2);
                }


                //strafe left
                drive.goToPosition(-0.5, 0.5, 0.5, -0.5, 600, "strafe left");


                // PARK ------------------------------------------------------>

                //moves robot to correct parking position
                if (detect1.getParkPosition() == ContourMultiScoreLeft1.ParkingPosition.LEFT) {
                    // go backwards a bit
                    drive.goToPosition(0.52, 0.5, 0.52, 0.5, 30, "go forwards a bit");
                    // strafe left
                    drive.goToPosition(-0.6, 0.6, 0.6, -0.6, 501, "strafe left");

                } else if (detect1.getParkPosition() == ContourMultiScoreLeft1.ParkingPosition.CENTER) {
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
                running = false;
            } else if (scoringPole && running) {
                // move backwards (away from pole)
                drive.goToPosition(-0.5, -0.52, -0.5, -0.52, 100, "go backwards");


                //moves robot to correct parking position
                drive.absTurnPID(Math.PI / 2);


                //}
                // PARK ------------------------------------------------------>

                //moves robot to correct parking position
                if (detect1.getParkPosition() == ContourMultiScoreLeft1.ParkingPosition.LEFT) {
                    // go forward
                    drive.goToPosition(0.62, 0.6, 0.62, 0.6, 880, "go forwards");
                    // strafe left
                    drive.goToPosition(-0.6, 0.6, 0.6, -0.6, 501, "strafe left");

                } else if (detect1.getParkPosition() == ContourMultiScoreLeft1.ParkingPosition.CENTER) {
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

                running = false;
            } else if (strafing && running) {

                //moves robot to correct parking position
                drive.absTurnPID(Math.PI / 2);


                //}
                // PARK ------------------------------------------------------>

                //moves robot to correct parking position
                if (detect1.getParkPosition() == ContourMultiScoreLeft1.ParkingPosition.LEFT) {
                    // go forward
                    drive.goToPosition(0.62, 0.6, 0.62, 0.6, 880, "go forwards");
                    // strafe left
                    drive.goToPosition(-0.6, 0.6, 0.6, -0.6, 501, "strafe left");

                } else if (detect1.getParkPosition() == ContourMultiScoreLeft1.ParkingPosition.CENTER) {
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
            running = false;
            safePark.set(false);
            this.strafing = false;
            this.scoringPole = false;
            this.findingTape = false;
            this.grabbingCone = false;

            score.goToPosition(0, 0.8);
            sleep(1000000);
        }
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

    public void stackPos(int pos) {

        if (pos == 5) {
            score.goToPosition(constants.getStackHeight() - 5, 0.7);
        } else if (pos == 4) {
            score.goToPosition(constants.getStack4() - 5, 0.7);
        } else if (pos == 3) {
            score.goToPosition(constants.getStack3() - 5, 0.7);

        } else if (pos == 2) {
            score.goToPosition(constants.getStack2() - 5, 0.7);

        }
        sleep(50);
    }

    public static Constants.Pipeline getNormalization() {
        return normalization;
    }
}
