package org.firstinspires.ftc.teamcode.Competition.MTI;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Competition.Interleagues.Common.Constants;
import org.firstinspires.ftc.teamcode.Competition.Interleagues.Common.Vector2D;
import org.firstinspires.ftc.teamcode.Competition.Interleagues.Subsystems.Used.MecDrive;
import org.firstinspires.ftc.teamcode.Competition.State.Subsystems.Current.MecDriveV2;


@TeleOp(name = "MTI Version2.5 Teleop")
public class NoSequenceTeleOp extends LinearOpMode {

    ScoringSystemNewest score;
    MecDriveV2 drive;
    ElapsedTime time = new ElapsedTime();
    Servo wheelieServo, cameraOdo;

    double wheeliePos = Constants.wheelieRetracted;

    double linkageToggleSpeed = 0.001;


    ColorRangeSensor distance;

    //PassivePower passive;

    volatile boolean autoLinkageFlag, grabFlag, shiftLinkageFlag, manualFlag, changeStackFlag, linkageUp, linkageDown, firstDpadUp, changeToggle;
    volatile boolean liftBrokenMode = false;
    volatile boolean optionsFlag = true;
    volatile boolean startFlag = true;

    Thread liftThread, linkageThread;

    //Enums for feed forward


    @Override
    public void runOpMode() throws InterruptedException {

        wheelieServo = hardwareMap.get(Servo.class, "wheelie");
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
        score.setGrabberPosition(Constants.open - 0.04);

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

                            score.setGrabberPosition(Constants.score);


                            //Do nothing during movement phase
                            //Reset to zero and no passive power
                            //score.moveToPosition(0, 0.5);
                            score.setLiftTarget(0);


                            score.setGrabberPosition(Constants.open - 0.04);

                            try {
                                sleep(300);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }

                            linkageDown = true;
                            //passive = PassivePower.ZERO;

                            //Open Grabber and reset linkage

                            //score.setLinkagePositionLogistic(Constants.linkageDownV2, 300);
                            //score.setLinkagePositionLogistic(0.8, 500);
                        } else {

                            score.setGrabberPosition(Constants.open - 0.04);
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


                        score.setGrabberPosition(Constants.grabbing);

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


                    /*else if((distance.getDistance(DistanceUnit.CM) < 2) && grabFlag) {
                        score.setGrabberPosition(Constants.grabbing);

                        grabFlag = false;
                        try {
                            sleep(200);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }



                    }
                    */


                    //TODO: see if need to fix this logic
                    //Auto linkage up logic after sensing a cone


                    //TODO: tune this (both raise and lower)
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


                    //Linkage up position
                    if (gamepad1.left_stick_button) {


                    }


                    //Manual open and close grabber
                    if (gamepad1.right_stick_button) {

                        linkageUp = true;
                        autoLinkageFlag = false;
                        wheelieServo.setPosition(wheeliePos);
                        /*if (score.getGrabberPosition() != Constants.open + 0.04) {
                            score.setGrabberPosition(Constants.open + 0.04);
                            try {
                                sleep(300);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                            grabFlag = true;
                        } else {
                            score.setGrabberPosition(Constants.grabbing);
                            try {
                                sleep(300);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                            grabFlag = false;
                        }
                        manualFlag = false;
                    }

                    if (!gamepad1.right_stick_button) {
                        manualFlag = true;
                    }*/

                    }


                    //Changing scoring modes (toggle)

                    if (gamepad1.y || gamepad1.a || gamepad1.x || gamepad1.b) {
                        if (gamepad1.y) {
                            score.setScoringMode(ScoringSystemNewest.ScoringMode.LOW);
                            wheeliePos = Constants.wheelieRetracted;


                        } else if (gamepad1.x) {
                            score.setScoringMode(ScoringSystemNewest.ScoringMode.MEDIUM);
                            wheeliePos = Constants.wheelieHigh;


                        } else if (gamepad1.b) {
                            score.setScoringMode(ScoringSystemNewest.ScoringMode.HIGH);
                            wheeliePos = Constants.wheelieHigh;


                        } else if (gamepad1.a) {
                            //Ultra
                            score.setScoringMode(ScoringSystemNewest.ScoringMode.ULTRA);
                            wheeliePos = Constants.wheelieRetracted;

                        }

                        wheelieServo.setPosition(wheeliePos);
                    }


                    //Manual slides (dpad right and left)
                    if (gamepad1.dpad_right) {
                        //passive = PassivePower.MOVEMENT;
                        score.setPower(1);
                        score.setLiftTarget(-1 * score.getRightEncoderPos());
                    } else if (gamepad1.dpad_left) {
                        //passive = PassivePower.MOVEMENT;
                        score.setPower(-0.55);
                        score.setLiftTarget(-1 * score.getRightEncoderPos());

                    } else {

                        if (score.getLiftTarget() < 2000 && Math.abs(score.getRightEncoderPos()) > 2000) {
                            score.newLiftPIDUpdate(0.55, false);
                            telemetry.addData("stuff", "slow");

                        } else if (score.getLiftTarget() != 0) {

                            if(score.getScoringMode() == ScoringSystemNewest.ScoringMode.MEDIUM){
                                score.newLiftPIDUpdate(0.8, false);
                            }else if(score.getScoringMode() == ScoringSystemNewest.ScoringMode.LOW){
                                score.newLiftPIDUpdate(1, false);
                            }else if(score.getScoringMode() == ScoringSystemNewest.ScoringMode.HIGH){
                                if(Math.abs(score.getRightEncoderPos()) < score.getLiftTarget()) {
                                    score.newLiftPIDUpdate(1, true);
                                }else{
                                    score.newLiftPIDUpdate(1, false);
                                }
                            }

                            telemetry.addData("stuff", "fast");

                        } else {
                            score.setPower(0);
                        }


                    }

                    telemetry.addData("stuff", "none");

                    telemetry.addData("target", score.getLiftTarget());
                    telemetry.addData("right", score.getRightEncoderPos());
                    telemetry.addData("left", score.getLeftEncoderPos());

                    telemetry.update();



                    /*else{

                        //Feedforward if slides are extended
                        if(score.isExtended() && (score.getScoringMode() == ScoringSystemNewest.ScoringMode.LOW || score.getScoringMode() == ScoringSystemNewest.ScoringMode.MEDIUM)){
                            passive = PassivePower.EXTENDED;
                        }else{
                            passive = PassivePower.ZERO;
                        }


                    }*/

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

                    /*if (gamepad2.dpad_up) {
                        score.setLinkagePosition(score.getLeftLinkage() + 0.001);
                    }else if (gamepad2.dpad_down) {
                        score.setLinkagePosition(score.getLeftLinkage() - 0.001);
                    }else if(gamepad2.dpad_left){
                        score.setLinkagePosition(score.getLeftLinkage() - 0.002);
                    }else if(gamepad2.dpad_right){
                        score.setLinkagePosition(score.getLeftLinkage() + 0.002);
                    }*/

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

                    if (score.getLiftTarget() == 0 && Math.abs(score.getRightEncoderPos()) < 2000 && score.getRightLinkage() != 0.15 && (score.getConeStack() == 1 || score.getConeStack() == 2) && autoLinkageFlag && !score.isExtended() && !startFlag) {
                        if(score.getConeStack() == 1) {
                            score.setLinkagePosition(0.15);
                        }else{
                            score.setLinkagePosition(0.22);
                        }
                    }


                }

            }
        };

        //CapThread
        /*capThread = new Thread(){


            @Override
            public void run() {
                while(opModeIsActive()){

                    //Capstone extension and retraction
                    if(gamepad2.right_trigger > 0 || gamepad2.left_trigger > 0) {
                        double val = gamepad2.right_trigger - gamepad2.left_trigger;
                        systems.setCapstoneExtensionPower(val);

                        //Fine tune with left and right bumper
                    } else if (gamepad2.right_bumper) {
                        systems.setCapstoneExtensionPower(0.25);
                    } else if (gamepad2.left_bumper) {
                        systems.setCapstoneExtensionPower(-0.25);
                    } else{
                        systems.setCapstoneExtensionPower(0);
                    }



                    //Setting y potion of cap mech
                    double yPos = systems.getYCapPosition();
                    systems.setXCapstoneRotatePower(gamepad2.right_stick_x);
                    systems.setYCapPosition(yPos - systems.map(gamepad2.right_stick_y, -1, 1, -0.0010, 0.0010));

                    //Reset the y servo
                    if (gamepad2.x) {
                        systems.zeroCap();

                        //Change the speed of x servo rotation
                    } else if (gamepad2.dpad_right && previousRight != gamepad2.dpad_right) {
                        systems.setXCapSpeedDivisor(10);
                    } else if (gamepad2.dpad_left && previousLeft != gamepad2.dpad_left) {
                        systems.setXCapSpeedDivisor(7);
                    }


                    previousLeft = gamepad2.dpad_left;
                    previousRight = gamepad2.dpad_right;
                    previousUp = gamepad2.dpad_up;
                    previousDown = gamepad2.dpad_down;
                }

                //On stop
                systems.zeroCap();
                systems.setXCapstoneRotatePower(0);
                systems.setCapstoneExtensionPower(0);




            }
        };
*/
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

                        wheelieServo.setPosition(wheeliePos);


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

                        //score.setLinkagePositionLogistic(Constants.linkageDownV2, 300, 100);

                        //TODO: fix cone stack logic
                        score.setLinkageConeStack(false);
                        linkageDown = false;
                    }
                }
            }
        };

        wheelieServo.setPosition(wheeliePos);


        waitForStart();

        //score.setLinkagePosition(Constants.linkageUpV2);


        //Starting Threads
        liftThread.start();
        //capThread.start();
        linkageThread.start();

        while (opModeIsActive()) {

            //double imuOrientation = drive.getTippingAngle();


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

            /*telemetry.addData("leftStickX", leftStickX);
            telemetry.addData("leftSticky", leftStickY);
            telemetry.addData("turn", gamepad1.right_stick_x);
            //telemetry.addData("tippingAngle", imuOrientation);
            telemetry.addData("Math.PI/64", Math.PI/64);

             */


            if (gamepad1.right_bumper) {
                drive.setPower(new Vector2D(-leftStickX * Constants.SPRINT_LINEAR_MODIFIER, -leftStickY * Constants.SPRINT_LINEAR_MODIFIER), -gamepad1.right_stick_x * 0.7, false);
            } else {

                //if(imuOrientation > -1.4 || imuOrientation < -1.465) {
                //drive.tippingUpdate(2000, imuOrientation);
                //}else {
                if (score.isExtended() && score.getScoringMode() != ScoringSystemNewest.ScoringMode.LOW) {
                    //Slow down when slides are extended
                    drive.setPower(new Vector2D(-leftStickX * 0.8, -leftStickY * 0.8), -gamepad1.right_stick_x * 0.45, false);
                } else {
                    drive.setPower(new Vector2D(-leftStickX * 0.8/* * Constants.NORMAL_LINEAR_MODIFIER*/, -leftStickY * 0.8/* * Constants.NORMAL_LINEAR_MODIFIER*/), -gamepad1.right_stick_x * 0.8, false);
                }
            }


                //Telemetry

            /*telemetry.addData("lMotor", -1 * score.getLeftEncoderPos());
            telemetry.addData("rMotor", score.getRightEncoderPos());
            telemetry.addData("distance: ", distance.getDistance(DistanceUnit.CM));
            telemetry.addData("distanceRed", distance.getNormalizedColors().red);
            telemetry.addData("distanceBlue", distance.getNormalizedColors().blue);
            telemetry.addData("autoLinkageFlag", autoLinkageFlag);
            telemetry.addData("grabbingFlag", grabFlag);
            telemetry.addData("manualFlag", manualFlag);
            telemetry.addData("shiftLinkageFlag", shiftLinkageFlag);
            telemetry.addData("extended", score.isExtended());
            //telemetry.addData("colorRed: ", color.getNormalizedColors().red);
            //telemetry.addData("colorBlue: ", color.getNormalizedColors().blue);
            telemetry.addData("rightServoTarget", score.getRightLinkage());
            telemetry.addData("leftServoTarget", score.getLeftLinkage());
            //telemetry.addData("passive", passive);
            telemetry.addData("coneStack", score.getConeStack());
            telemetry.addData("rip robot", liftBrokenMode);
            telemetry.update();
*/

            }


            //Stop
            drive.simpleBrake();

            //score.setLinkagePositionLogistic(0.25, 500);
            score.setLinkagePositionLogistic(Constants.linkageDownV2, 300, 100);
            //score.setLinkagePositionLogistic(0.8, 500);


            score.setGrabberPosition(Constants.open - 0.04);
    }
}
