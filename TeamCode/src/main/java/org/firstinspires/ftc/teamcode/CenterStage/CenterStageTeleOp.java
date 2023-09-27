package org.firstinspires.ftc.teamcode.CenterStage;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.Competition.Interleagues.Common.Constants;
import org.firstinspires.ftc.teamcode.Competition.Interleagues.Common.*;
import org.firstinspires.ftc.teamcode.Competition.State.Subsystems.Current.*;
import org.firstinspires.ftc.teamcode.Competition.MTI.*;





@TeleOp(name = "CenterStageTeleOp")
public class CenterStageTeleOp extends LinearOpMode {

    ScoringSystemNewest score;
    MecDriveV2 drive;
    ElapsedTime time = new ElapsedTime();
    Servo cameraOdo;

    double wheeliePos = Constants.wheelieRetracted;

    double tempTime = 0;
    double linkageToggleSpeed = 0.001;


    ColorRangeSensor distance;

    //PassivePower passive;

    volatile boolean autoLinkageFlag, grabFlag, shiftLinkageFlag, manualFlag, changeStackFlag, linkageUp, linkageDown, firstDpadUp, changeToggle;
    volatile boolean liftBrokenMode = false;
    volatile boolean optionsFlag = true;
    volatile boolean startFlag = true;
    volatile boolean groundFlag = false;

    Thread liftThread, linkageThread;

    //Enums for feed forward


    @Override
    public void runOpMode() throws InterruptedException {


        cameraOdo = hardwareMap.get(Servo.class, "camera");
        //Initializing flags
        autoLinkageFlag = true;
        grabFlag = true;
        shiftLinkageFlag = true;
        manualFlag = true;
        linkageDown = false;
        linkageUp = false;
        firstDpadUp = true;
        changeToggle = true;


        //Feed forward is going to be off

        cameraOdo.setPosition(0);


        score = new ScoringSystemNewest(hardwareMap, telemetry, true, time);
        //robot = new Robot(hardwareMap);

        drive = new MecDriveV2(hardwareMap, false, telemetry, time);
        //systems = new EndgameSystems(hardwareMap);


        //score.setLinkagePositionLogistic(Constants.linkageDown, 500);
        score.setGrabberPosition(0.13);

        distance = hardwareMap.get(ColorRangeSensor.class, "distance");


        //Color sensor gain values
        //color.setGain(300);
        distance.setGain(230);


        //Lift Thread
        liftThread = new Thread() {
            @Override
            public void run() {
                while (opModeIsActive()) {

                    //Lift up to scoring position
                    if (gamepad1.left_trigger > 0.1) {
                        //score.setPower(0.2);
                        if (score.getScoringMode() != ScoringSystemNewest.ScoringMode.ULTRA && !liftBrokenMode) {

                            score.commandAutoGoToPosition();

                            if (score.getScoringMode() == ScoringSystemNewest.ScoringMode.LOW) {
                                score.setLinkagePosition(0.7);
                            }
                            /*else {
                                score.setLinkagePosition(Constants.linkageScoreV2 + 0.05);
                            }*/
                            //passive = PassivePower.EXTENDED;
                        } else if (score.getScoringMode() == ScoringSystemNewest.ScoringMode.ULTRA) {
                            score.setLinkagePosition(0.15);
                        }

                    }/*else {
                        if(passive == PassivePower.EXTENDED){
                            score.setPowerSingular(0.23);
                        }else if(passive == PassivePower.ZEROf){
                            score.setPower(0);
                        }
                    }*/


                    //Scoring feature
                    if (gamepad1.right_trigger > 0.1) {
                        startFlag = false;

                        if (score.getScoringMode() != ScoringSystemNewest.ScoringMode.ULTRA) {

                            if (Math.abs(score.getRightEncoderPos()) > 2000) {
                                if (score.getScoringMode() == ScoringSystemNewest.ScoringMode.LOW) {
                                    score.setLinkagePosition(Constants.linkageScoreV2 + 0.12);
                                } else if(score.getScoringMode() == ScoringSystemNewest.ScoringMode.MEDIUM) {
                                    score.setLinkagePosition(Constants.linkageScoreV2 + 0.08);
                                }else{
                                    score.setLinkagePosition(Constants.linkageScoreV2 + 0.05);
                                }
                            }

                            try {
                                Thread.currentThread().sleep(100);
                            } catch (InterruptedException e) {
                                throw new RuntimeException(e);
                            }

                            score.setGrabberPosition(0.25);
                            score.setLiftTarget(0);


                            score.setGrabberPosition(0.13);

                            try {
                                sleep(300);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }

                            linkageDown = true;

                        } else {

                            score.setGrabberPosition(0.1);
                            tempTime = time.milliseconds();
                            groundFlag = true;


                            try {
                                sleep(700);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }



                        }

                        if (liftBrokenMode) {
                            try {
                                sleep(2000);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                        }

                        //TODO: fix this
                        score.lowerConeStack();


                        //Resetting flags
                        autoLinkageFlag = true;
                        grabFlag = true;

                        try {
                            Thread.currentThread().sleep(500);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }

                        //Not extended anymore
                        score.setExtended(false);

                        //Automated Grab
                    } else if ((distance.getNormalizedColors().red > 0.90 || distance.getNormalizedColors().blue > 0.90) && autoLinkageFlag) {


                        score.setGrabberPosition(0.37);

                        grabFlag = false;

                        try {
                            Thread.currentThread().sleep(250);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }

                        if (score.getScoringMode() == ScoringSystemNewest.ScoringMode.ULTRA) {
                            try {
                                sleep(400);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }

                        }


                        linkageUp = true;
                        autoLinkageFlag = false;


                    }


                    //Linkage stack cone heights with dpad up and down
                    if ((gamepad1.dpad_up || gamepad1.dpad_down) && changeStackFlag) {

                        //Raise linkage by height of a cone (max height of 5)
                        if (gamepad1.dpad_up) {
                            score.setConeStack(4);
                            score.setLinkageConeStack(false);
                            changeStackFlag = false;

                            //Lower linkage by height of a cone (min height of 1)
                        } else if (gamepad1.dpad_down) {
                            score.lowerConeStack();
                            score.setLinkageConeStack(false);
                            changeStackFlag = false;

                        }

                        try {
                            sleep(200);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }

                    }
                    if (!gamepad1.dpad_down && !gamepad1.dpad_up) {
                        changeStackFlag = true;
                    }



                    //Linkage up
                    //TODO: see if we want to put auto grabber close
                    if (gamepad1.right_stick_button) {

                        linkageUp = true;
                        autoLinkageFlag = false;

                    }


                    //Changing scoring modes (toggle)

                    if (gamepad1.y || gamepad1.a || gamepad1.x || gamepad1.b) {
                        if (gamepad1.y) {
                            score.setScoringMode(ScoringSystemNewest.ScoringMode.LOW);


                        } else if (gamepad1.x) {
                            score.setScoringMode(ScoringSystemNewest.ScoringMode.MEDIUM);


                        } else if (gamepad1.b) {
                            score.setScoringMode(ScoringSystemNewest.ScoringMode.HIGH);


                        } else if (gamepad1.a) {
                            //Ultra
                            score.setScoringMode(ScoringSystemNewest.ScoringMode.ULTRA);

                        }

                    }


                    //Manual slides (dpad right and left)
                    if (gamepad1.dpad_right) {
                        score.setPower(1);
                        score.setLiftTarget(-1 * score.getRightEncoderPos());
                    } else if (gamepad1.dpad_left) {
                        score.setPower(-0.55);
                        score.setLiftTarget(-1 * score.getRightEncoderPos());

                    } else {

                        if (score.getLiftTarget() != 0) {

                            //TODO: Probably will need to change how we are doing heights
                            if(score.getScoringMode() == ScoringSystemNewest.ScoringMode.MEDIUM){
                                score.newLiftPIDUpdate(0.8, false);
                            }else if(score.getScoringMode() == ScoringSystemNewest.ScoringMode.LOW){
                                score.newLiftPIDUpdate(1, false);
                            }else if(score.getScoringMode() == ScoringSystemNewest.ScoringMode.HIGH){
                                if(Math.abs(score.getRightEncoderPos()) < score.getLiftTarget() && Math.abs(score.getLeftEncoderPos()) < score.getLiftTarget()) {
                                    score.newLiftPIDUpdate(1, true);
                                }else{
                                    score.newLiftPIDUpdate(1, false);
                                }
                            }


                        } else {
                            score.newLiftPIDUpdate(0.62, false);

                        }


                    }


                    telemetry.update();


                    if (gamepad1.left_bumper && optionsFlag) {
                        optionsFlag = false;
                        liftBrokenMode = !liftBrokenMode;

                        if (liftBrokenMode) {
                            gamepad1.rumble(1500);
                        } else {
                            gamepad1.rumble(200);
                        }
                    }
                    if (!gamepad1.left_bumper) {
                        optionsFlag = true;
                    }


                    if ((gamepad2.dpad_up || gamepad2.dpad_down) && changeToggle) {
                        if (gamepad2.dpad_up) {
                            score.setLinkagePosition(score.getLeftLinkage() + 0.025);

                        } else {
                            score.setLinkagePosition(score.getLeftLinkage() - 0.025);


                        }

                        changeToggle = false;


                    }

                    if (!gamepad2.dpad_up && !gamepad2.dpad_down) {
                        changeToggle = true;
                    }

                    if(time.milliseconds() - tempTime > 3000 && groundFlag){
                        score.setGrabberPosition(0.13);

                        groundFlag = false;
                    }



                }

            }
        };


        //Logistic Linkage thread
        linkageThread = new Thread() {

            @Override
            public void run() {
                while (opModeIsActive()) {
                    if (linkageUp) {

                        try {
                            Thread.currentThread().sleep(100);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                        if (score.getScoringMode() == ScoringSystemNewest.ScoringMode.ULTRA) {
                            score.setLinkagePosition(0.25);
                        } else if (liftBrokenMode) {
                            score.setLinkagePosition(0.54);
                        } else {
                            score.setLinkagePosition(Constants.linkageScoreV2 - 0.01);
                        }



                        if (score.getScoringMode() == ScoringSystemNewest.ScoringMode.LOW || score.getScoringMode() == ScoringSystemNewest.ScoringMode.MEDIUM) {
                            score.commandAutoGoToPosition();
                            if (score.getScoringMode() == ScoringSystemNewest.ScoringMode.LOW) {
                                score.setLinkagePosition(0.7);
                            }
                            score.setExtended(true);

                        }
                        linkageUp = false;
                    } else if (linkageDown) {

                        score.setLinkagePosition(Constants.linkageUpV2);
                        try {
                            Thread.currentThread().sleep(70);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }

                        score.setLinkagePosition(0.35);


                        //TODO: fix cone stack logic
                        score.setLinkageConeStack(false);
                        linkageDown = false;
                    }
                }
            }
        };



        waitForStart();



        //Starting Threads
        liftThread.start();
        linkageThread.start();

        while (opModeIsActive()) {


            //N S E W Drive
            double leftStickX = gamepad1.left_stick_x;
            double leftStickY = gamepad1.left_stick_y;

            if (Math.abs(leftStickX) > Math.abs(leftStickY)) {
                leftStickY = 0;

            } else if (Math.abs(leftStickY) > Math.abs(leftStickX)) {
                leftStickX = 0;

            } else {
                leftStickY = 0;
                leftStickX = 0;
            }


            if (gamepad1.right_bumper) {
                drive.setPower(new Vector2D(-leftStickX * Constants.SPRINT_LINEAR_MODIFIER, -leftStickY * Constants.SPRINT_LINEAR_MODIFIER), -gamepad1.right_stick_x * 0.7, false);
            } else {

                if (score.isExtended() && score.getScoringMode() != ScoringSystemNewest.ScoringMode.LOW) {
                    //Slow down when slides are extended
                    drive.setPower(new Vector2D(-leftStickX * 0.8, -leftStickY * 0.8), -gamepad1.right_stick_x * 0.45, false);
                } else {
                    drive.setPower(new Vector2D(-leftStickX * 0.8/* * Constants.NORMAL_LINEAR_MODIFIER*/, -leftStickY * 0.8/* * Constants.NORMAL_LINEAR_MODIFIER*/), -gamepad1.right_stick_x * 0.8, false);
                }
            }


        }


            //Stop
            drive.simpleBrake();
            score.setLinkagePositionLogistic(Constants.linkageDownV2, 300, 100);
            score.setGrabberPosition(0.13);
    }
}
