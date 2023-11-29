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

//Gamepad 1
/*Left Stick: Sprint
Dpad Left = Manual Up
Dpad Right = Manual Down
a = low 
x = medium 
y = high
b = encoder reset

left Trigger = Auto Lift
rightTrigger = score
start = manual linkage
back = manual grabber
right bumper = intake*/

//Game Pad 2
/*Dpad Up = Climber Auto + Manual Up
Dpad Down = Climber Manual Down */





@TeleOp(name = "CenterStageTeleOp")
public class CenterStageTeleOp extends LinearOpMode {

    ScoringSystem score;
    MecDriveV2 drive;
    ElapsedTime time = new ElapsedTime();

    //PassivePower passive;
    volatile boolean linkageUp, linkageDown, climbed, movingUp;

    Thread liftThread, linkageThread;

 //   boolean swappied = false;

    //Enums for feed forward

    @Override
    public void runOpMode() throws InterruptedException {
        //Initializing flags

        linkageDown = false;
        linkageUp = false;
        climbed = false;

        score = new ScoringSystem(hardwareMap, telemetry, time);

        drive = new MecDriveV2(hardwareMap, false, telemetry, time);

        //Start with linkage down and grabber open
        score.setLinkagePositionLogistic(Constants.LINKAGE_DOWN, 500, 100);
        score.setGrabberPosition(Constants.OPEN);
        score.setBumperPixelRelease(Constants.AUTO_SCORING_CLAMP_OPEN);
        score.setBumperPixelRelease(Constants.AUTO_SCORING_CLAMP_CLOSED);

        //Lift Thread
        liftThread = new Thread() {
            @Override
            public void run() {
                //Linkage up anshe sd down
                boolean startFlag = false;

                //Grabber open and close
                boolean backFlag = false;

                //Rest the encoders
                boolean restEncoderFlag = false;

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
                    //If you pressed the start button and the flag is false the continue
                    //TODO: see if we want to put auto grabber close
                    if (gamepad1.start && !startFlag) {

                        //If linkage is down then move it up
                        if (score.getLeftLinkage() == Constants.LINKAGE_DOWN) {
                            linkageUp = true;

                            //If the linkage is up then move it down
                        } else {
                            linkageDown = true;
                        }

                        //Set flag to true so that it won't run over and over again when the button is pressed
                        //Basically stating that you already pressed the button
                        startFlag = true;
                    }

                    //If you release the start button then set the flag to false
                    //Basically stating that you released the button
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
                        backFlag = false;
                    }

                    //Changing scoring modes (toggle)
                    if (gamepad1.a) {
                        score.setScoringMode(ScoringSystem.ScoringMode.LOW);


                    } else if (gamepad1.x) {
                        score.setScoringMode(ScoringSystem.ScoringMode.MEDIUM);


                    } else if (gamepad1.y) {
                        score.setScoringMode(ScoringSystem.ScoringMode.HIGH);


                    }

                    //Reset Encoder if slides bully us
                    if (gamepad1.b && !restEncoderFlag) {
                        score.reset();
                        score.setLiftTarget(0);
                        restEncoderFlag = true;
                    }
                    if (!gamepad1.b) {
                        restEncoderFlag = false;
                    }

                    //Manual slides (dpad right goes up and left goes down)
                    if (gamepad1.dpad_right) {
                        score.setPower(0.7);
                        score.setLiftTarget(-1 * score.getRightEncoderPos());
                    } else if (gamepad1.dpad_left) {

                        score.setPower(-0.7);
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
                boolean right_bumper = false;

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
                        if (score.isExtended()) {
                            score.commandAutoGoToPosition();
                        } else {
                            movingUp = true;
                            score.setLiftTarget(100);
                            try {
                                Thread.currentThread().sleep(300);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                            //score.setPower(0.2);
                            score.setLinkagePositionLogistic(0.6, 1000, 100);

                            score.setGrabberPosition(Constants.GRABBING);
                            score.commandAutoGoToPosition();

                            score.setExtended(true);
                        }
                    }

                    //Scoring feature
                    if (gamepad1.right_trigger > 0.1) {
                        score.setGrabberPosition(Constants.OPEN);

                        try {
                            Thread.currentThread().sleep(500);
                        } catch (InterruptedException e) {
                            throw new RuntimeException(e);
                        }

                        score.setLiftTarget(score.getLiftTarget() + 50);

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
                    if (gamepad1.right_bumper && score.getLeftLinkage() <= Constants.LINKAGE_DOWN) {
                        score.setGrabberPosition(Constants.OPEN);
                        score.setLinkagePosition(Constants.LINKAGE_INTAKE);
                        score.setIntakeLiftPos(Constants.INTAKE_LINKAGE_DOWN);
                        score.setIntakePower(Constants.INTAKE_SPEED);
                        right_bumper = true;
                    } else if (right_bumper) {
                        score.setGrabberPosition(Constants.GRABBING);
                        score.setIntakePower(0);
                        try {
                            Thread.currentThread().sleep(250);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                        score.setLinkagePosition(Constants.LINKAGE_DOWN);
                        score.setIntakeLiftPos(Constants.INTAKE_LINKAGE_UP);
                        right_bumper = false;
                    }

                    // Extend climb mechanism when dpad up pressed
                    if (gamepad2.dpad_up && !climbed) {
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
                    } else if (gamepad2.dpad_up) {
                        score.setClimbPower(0.3);
                    } else if (gamepad2.dpad_down) {
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
        score.setBumperPixelRelease(Constants.AUTO_SCORING_CLAMP_OPEN);


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

            if (gamepad1.left_stick_button) {
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
        //    score.setGrabberPosition(Constants.OPEN);
    }
}
