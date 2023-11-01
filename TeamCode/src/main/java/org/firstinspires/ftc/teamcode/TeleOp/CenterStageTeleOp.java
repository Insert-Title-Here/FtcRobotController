package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.Common.Constants;
import org.firstinspires.ftc.teamcode.Common.MecDriveV2;
import org.firstinspires.ftc.teamcode.Common.ScoringSystem;
import org.firstinspires.ftc.teamcode.Common.Vector2D;

@TeleOp(name = "CenterStageTeleOp")
public class CenterStageTeleOp extends LinearOpMode {

    ScoringSystem score;
    MecDriveV2 drive;
    ElapsedTime time = new ElapsedTime();

    //PassivePower passive;
    volatile boolean linkageUp, linkageDown, climbed, movingUp;

    Thread liftThread, linkageThread;

    boolean swappied = false;

    //Enums for feed forward

    @Override
    public void runOpMode() throws InterruptedException {
        //Initializing flags

        linkageDown = false;
        linkageUp = false;
        climbed = false;

        score = new ScoringSystem(hardwareMap, telemetry, time);

        drive = new MecDriveV2(hardwareMap, false, telemetry, time);

        score.setLinkagePositionLogistic(Constants.LINKAGE_DOWN, 500, 100);
        score.setGrabberPosition(Constants.OPEN);

        //Lift Thread
        liftThread = new Thread() {
            @Override
            public void run() {
                //Linkage up and down
                boolean startFlag = false;

                //Grabber open and close
                boolean backFlag = false;

                while (opModeIsActive()) {

                    /*
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

                     */

                    //Manual Linkage up and down
                    //TODO: see if we want to put auto grabber close
                    if (gamepad1.start && !startFlag) {
                        if (score.getLeftLinkage() == Constants.LINKAGE_DOWN) {
                            linkageUp = true;
                        } else {
                            linkageDown = true;
                        }
                        startFlag = true;
                    }
                    if (!gamepad1.start) {
                        startFlag = false;
                    }

                    //Manual Grabbing and open
                    if (gamepad1.back && !backFlag) {
                        if (score.getGrabberPosition() == Constants.GRABBING) {
                            score.setGrabberPosition(Constants.OPEN);
                        } else {
                            score.setGrabberPosition(Constants.GRABBING);
                        }
                        backFlag = true;
                    }
                    if (!gamepad1.back) {
                        startFlag = false;
                    }


                    //Changing scoring modes (toggle)
                    if (gamepad1.a) {
                        score.setScoringMode(ScoringSystem.ScoringMode.LOW);


                    } else if (gamepad1.x) {
                        score.setScoringMode(ScoringSystem.ScoringMode.MEDIUM);


                    } else if (gamepad1.y) {
                        score.setScoringMode(ScoringSystem.ScoringMode.HIGH);


                    }


                    //Manual slides (dpad right goes up and left goes down)
                    if (gamepad1.dpad_right) {
                        score.setPower(0.5);
                        score.setLiftTarget(-1 * score.getRightEncoderPos());
                    } else if (gamepad1.dpad_left) {
                        score.setPower(-0.5);
                        score.setLiftTarget(-1 * score.getRightEncoderPos());

                    } else {

                        if (score.getLeftEncoderPos() * -1 < score.getLiftTarget() && score.getLiftTarget() != 0) {
                            //score.newLiftPIDUpdate(0.8, false);
                            score.liftUpdateNoPID(0.8, Constants.LIFT_F);
                        } else if (score.getLiftTarget() == 0){
                            score.liftUpdateNoPID(0.4, 0);
                        } else {
                            score.liftUpdateNoPID(0.4, Constants.LIFT_F);
                        }

                    }


                    telemetry.addData("climb", score.getClimbPosition());
                    telemetry.addData("left", score.getLeftEncoderPos());
                    telemetry.addData("right", score.getRightEncoderPos());
                    telemetry.addData("Target", score.getLiftTarget());
                    telemetry.update();
                }

            }
        };


        //Logistic Linkage thread
        linkageThread = new Thread() {

            @Override
            public void run() {
                boolean leftBumper = false;

                while (opModeIsActive()) {
                    /*if (gamepad1.b) {
                        score.releaseAirplane();
                    }

                     */

                    if(movingUp && -1 * score.getLeftEncoderPos() >= score.getLiftTarget()/2) {
                        linkageUp = true;
                        movingUp = false;
                    }
                    //Lift up to scoring position if climber has not been activated
                    if (gamepad1.left_trigger > 0.1 && !climbed) {
                        movingUp = true;
                        //score.setPower(0.2);
                        score.setLinkagePositionLogistic(0.6, 1000, 100);
                        try {
                            Thread.currentThread().sleep(750);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }

                        score.setGrabberPosition(Constants.GRABBING);
                        score.commandAutoGoToPosition();

                        score.setExtended(true);
                    }

                    //Scoring feature
                    if (gamepad1.right_trigger > 0.1) {
                        score.setGrabberPosition(Constants.OPEN);

                        try {
                            Thread.currentThread().sleep(500);
                        } catch (InterruptedException e) {
                            throw new RuntimeException(e);
                        }

                        score.setGrabberPosition(Constants.GRABBING);

                        score.setLinkagePositionLogistic(0.3, 500, 100);

                        score.setLiftTarget(100);

                        try {
                            sleep(200);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }

                        score.setLinkagePositionLogistic(Constants.LINKAGE_DOWN, 750, 100);

                        score.setLiftTarget(0);

                        //TODO: fix this
                        //score.lowerConeStack();




                        try {
                            Thread.currentThread().sleep(500);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }

                        //Not extended anymore
                        score.setExtended(false);
                    }

                    // Intake when LB pressed
                    if (gamepad1.left_bumper && score.getLeftLinkage() <= Constants.LINKAGE_DOWN) {
                        score.setGrabberPosition(Constants.OPEN);
                        score.setLinkagePosition(Constants.LINKAGE_INTAKE);
                        score.setIntakeLiftPos(Constants.INTAKE_LINKAGE_DOWN);
                        score.setIntakePower(Constants.INTAKE_SPEED);
                        leftBumper = true;
                    } else if (leftBumper) {
                        score.setGrabberPosition(Constants.GRABBING);
                        score.setIntakePower(0);
                        try {
                            Thread.currentThread().sleep(250);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                        score.setLinkagePosition(Constants.LINKAGE_DOWN);
                        score.setIntakeLiftPos(Constants.INTAKE_LINKAGE_UP);
                        leftBumper = false;
                    }

                    // Extend climb mechanism when dpad up pressed
                    if (gamepad1.dpad_up) {
                        // don't extend if lift or linkage is up
                        if (score.getLeftLinkage() > Constants.LINKAGE_DOWN || score.getLiftTarget() > 50) {
                            score.setLinkagePositionLogistic(Constants.LINKAGE_DOWN, 1000, 100);
                            score.setLiftTarget(0);

                            try {
                                Thread.currentThread().sleep(2000);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }

                        }

                        score.climbToPosition(Constants.CLIMB_HEIGHT, 1);
                        // flag to lock scoring system after climber is extended
                        climbed = true;
                    }
                    // lift robot if dpad down is pressed
                    if (gamepad1.dpad_down) {
                        score.setClimbPower(-1);
                    } else {
                        score.setClimbPower(0);
                    }

                    // only move linkage to up  position if climber isn't extended
                    if (linkageUp && !climbed) {

                        try {
                            Thread.currentThread().sleep(100);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }

                        score.setLinkagePositionLogistic(Constants.LINKAGE_UP, 1000, 100);

                        /*
                        if (score.getScoringMode() == ScoringSystem.ScoringMode.LOW || score.getScoringMode() == ScoringSystem.ScoringMode.MEDIUM) {
                            score.commandAutoGoToPosition();
                            if (score.getScoringMode() == ScoringSystem.ScoringMode.LOW) {
                                score.setLinkagePositionLogistic(0.7);
                            }
                            score.setExtended(true);

                        }

                         */

                        linkageUp = false;
                    } else if (linkageDown) {

                        score.setLinkagePositionLogistic(Constants.LINKAGE_DOWN, 1000, 100);


                        //TODO: fix cone stack logic
                        //score.setLinkageConeStack(false);
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

            // Uncomment this to lock to cardinal directions
            /*
            if (Math.abs(leftStickX) > Math.abs(leftStickY)) {
                leftStickY = 0;

            } else if (Math.abs(leftStickY) > Math.abs(leftStickX)) {
                leftStickX = 0;

            } else {
                leftStickY = 0;
                leftStickX = 0;
            }

             */
            boolean isOpen;
/*
        Neha finish this

            if (gamepad1.b && !swappied) {
                if (score.getGrabberPosition() == Constants.GRABBING) {
                    score.setGrabberPosition(Constants.OPEN);
                } else {
                    score.setGrabberPosition(Constants.GRABBING);
                }
                backFlag = true;
            }
            if (!gamepad1.b) {
                startFlag = false;
            }


 */
            if (gamepad1.right_bumper) {
                drive.setPower(new Vector2D(leftStickX * Constants.SPRINT_LINEAR_MODIFIER, leftStickY * Constants.SPRINT_LINEAR_MODIFIER), gamepad1.right_stick_x * Constants.SPRINT_ROTATIONAL_MODIFIER, false);
            }
            else {

                if (score.isExtended() && score.getScoringMode() != ScoringSystem.ScoringMode.LOW) {
                    //Slow down when slides are extended
                    drive.setPower(new Vector2D(leftStickX * Constants.EXTENDED_LINEAR_MODIFIER, leftStickY * Constants.EXTENDED_LINEAR_MODIFIER), gamepad1.right_stick_x * Constants.EXTENDED_ROTATIONAL_MODIFIER, false);
                } else {
                    drive.setPower(new Vector2D(leftStickX * Constants.NORMAL_LINEAR_MODIFIER, leftStickY * Constants.NORMAL_LINEAR_MODIFIER), gamepad1.right_stick_x * Constants.NORMAL_ROTATIONAL_MODIFIER, false);
                }
            }


        }


            //Stop
            drive.simpleBrake();
            score.setLinkagePositionLogistic(Constants.LINKAGE_DOWN, 300, 100);
            score.setGrabberPosition(Constants.OPEN);
    }
}
