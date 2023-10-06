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
    double tempTime = 0;
    double linkageToggleSpeed = 0.001;



    //PassivePower passive;

    volatile boolean /*, changeStackFlag,*/ linkageUp, linkageDown/*, changeToggle*/;
    volatile boolean liftBrokenMode = false;
    Thread liftThread, linkageThread;

    //Enums for feed forward


    @Override
    public void runOpMode() throws InterruptedException {


        //Initializing flags
        linkageDown = false;
        linkageUp = false;
        //changeToggle = true;

        score = new ScoringSystem(hardwareMap, telemetry, time);

        drive = new MecDriveV2(hardwareMap, false, telemetry, time);

        score.setLinkagePositionLogistic(Constants.LINKAGE_DOWN, 500, 100);
        score.setGrabberPosition(Constants.OPEN);

        //Lift Thread
        liftThread = new Thread() {
            @Override
            public void run() {
                while (opModeIsActive()) {

                    //Lift up to scoring position
                    if (gamepad1.left_trigger > 0.1) {
                        //score.setPower(0.2);
                        linkageUp = true;

                        try {
                            Thread.currentThread().sleep(650);
                        } catch (InterruptedException e) {
                            throw new RuntimeException(e);
                        }

                        score.commandAutoGoToPosition();
                        score.setExtended(true);
                    }/*else {
                        if(passive == PassivePower.EXTENDED){
                            score.setPowerSingular(0.23);
                        }else if(passive == PassivePower.ZEROf){
                            score.setPower(0);
                        }
                    }*/


                    //Scoring feature
                    if (gamepad1.right_trigger > 0.1) {
                        score.setGrabberPosition(Constants.OPEN);

                        try {
                            Thread.currentThread().sleep(500);
                        } catch (InterruptedException e) {
                            throw new RuntimeException(e);
                        }

                        linkageDown = true;

                        try {
                            sleep(300);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }


                        score.setLiftTarget(0);



                        if (liftBrokenMode) {
                            try {
                                sleep(2000);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                        }

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

                    //TODO: stack stuff

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



                    //Linkage up
                    //TODO: see if we want to put auto grabber close
                    if (gamepad1.right_stick_button) {


                        linkageUp = true;

                    }


                    //Changing scoring modes (toggle)

                    if (gamepad1.y) {
                        score.setScoringMode(ScoringSystem.ScoringMode.LOW);


                    } else if (gamepad1.x) {
                        score.setScoringMode(ScoringSystem.ScoringMode.MEDIUM);


                    } else if (gamepad1.b) {
                        score.setScoringMode(ScoringSystem.ScoringMode.HIGH);


                    } else if (gamepad1.a) {
                        //Ultra
                        score.setScoringMode(ScoringSystem.ScoringMode.ULTRA);

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
                            score.newLiftPIDUpdate(0.8, false);
                        } else {
                            score.newLiftPIDUpdate(0.62, false);

                        }


                    }


                    telemetry.update();

                    //TODO: lift broken?
                    /*
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

                     */


                    /*if ((gamepad2.dpad_up || gamepad2.dpad_down) && changeToggle) {
                        if (gamepad2.dpad_up) {
                            score.setLinkagePosition(score.getLeftLinkage() + 0.025);

                        } else {
                            score.setLinkagePosition(score.getLeftLinkage() - 0.025);


                        }

                        changeToggle = false;


                    }

                    if (!gamepad2.dpad_up && !gamepad2.dpad_down) {
                        changeToggle = true;
                    }*/




                }

            }
        };


        //Logistic Linkage thread
        linkageThread = new Thread() {

            @Override
            public void run() {

                boolean leftBumper = false;

                while (opModeIsActive()) {

                    if (gamepad1.left_bumper && score.getLeftLinkage() <= Constants.LINKAGE_DOWN) {
                        score.setGrabberPosition(Constants.OPEN);
                        score.setLinkagePosition(Constants.LINKAGE_INTAKE);
                        score.setIntakeLiftPos(Constants.INTAKE_LINKAGE_DOWN);
                        score.setIntakePower(1);
                        leftBumper = true;
                    } else if (leftBumper) {
                        score.setGrabberPosition(Constants.GRABBING);
                        score.setLinkagePosition(Constants.LINKAGE_DOWN);
                        score.setIntakeLiftPos(Constants.INTAKE_LINKAGE_UP);

                        score.setIntakePower(0);
                        leftBumper = false;
                    }

                    if (linkageUp) {

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

            if (Math.abs(leftStickX) > Math.abs(leftStickY)) {
                leftStickY = 0;

            } else if (Math.abs(leftStickY) > Math.abs(leftStickX)) {
                leftStickX = 0;

            } else {
                leftStickY = 0;
                leftStickX = 0;
            }


            if (gamepad1.right_bumper) {
                drive.setPower(new Vector2D(-leftStickX * Constants.SPRINT_LINEAR_MODIFIER, -leftStickY * Constants.SPRINT_LINEAR_MODIFIER), gamepad1.right_stick_x * Constants.SPRINT_ROTATIONAL_MODIFIER, false);
            } else {

                if (score.isExtended() && score.getScoringMode() != ScoringSystem.ScoringMode.LOW) {
                    //Slow down when slides are extended
                    drive.setPower(new Vector2D(-leftStickX * Constants.EXTENDED_LINEAR_MODIFIER, -leftStickY * Constants.EXTENDED_LINEAR_MODIFIER), gamepad1.right_stick_x * Constants.EXTENDED_ROTATIONAL_MODIFIER, false);
                } else {
                    drive.setPower(new Vector2D(-leftStickX * Constants.NORMAL_LINEAR_MODIFIER, -leftStickY * Constants.NORMAL_LINEAR_MODIFIER), gamepad1.right_stick_x * Constants.NORMAL_ROTATIONAL_MODIFIER, false);
                }
            }


        }


            //Stop
            drive.simpleBrake();
            score.setLinkagePositionLogistic(Constants.LINKAGE_DOWN, 300, 100);
            score.setGrabberPosition(Constants.OPEN);
    }
}
