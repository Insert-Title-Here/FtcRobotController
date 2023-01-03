package org.firstinspires.ftc.teamcode.V2.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.League1.Common.Constants;
import org.firstinspires.ftc.teamcode.League1.Common.Vector2D;
import org.firstinspires.ftc.teamcode.League1.Subsystems.MecDrive;
import org.firstinspires.ftc.teamcode.V2.NewSubsystem.ScoringSystemV2;
import org.firstinspires.ftc.teamcode.V2.NewSubsystem.ScoringSystemV2EpicLift;


@TeleOp (name = "KevinGodModeV2EpicLift")
public class QuadMotorLift extends LinearOpMode {

    Constants constants = new Constants();
    ScoringSystemV2EpicLift score;
    MecDrive drive;


    ColorRangeSensor distance, color;

    PassivePower passive;

    volatile boolean autoLinkageFlag, grabFlag, shiftLinkageFlag, manualFlag, changeStackFlag, linkageUp, linkageDown, firstDpadUp;
    volatile boolean liftBrokenMode = false;
    volatile boolean optionsFlag = true;

    Thread liftThread, linkageThread;

    //Enums for feed forward
    public enum PassivePower{
        //Feed forward is on
        EXTENDED,

        //Nothing
        MOVEMENT,

        //Power is set to 0
        ZERO,
    }


    @Override
    public void runOpMode() throws InterruptedException {
        //Initializing flags
        autoLinkageFlag = true;
        grabFlag = true;
        shiftLinkageFlag = true;
        manualFlag = true;
        linkageDown = false;
        linkageUp = false;
        firstDpadUp = true;

        ElapsedTime time = new ElapsedTime();


        //Feed forward is going to be off
        passive = PassivePower.ZERO;

        score = new ScoringSystemV2EpicLift(hardwareMap, constants, telemetry);
        //robot = new Robot(hardwareMap);
        drive = new MecDrive(hardwareMap,false, telemetry);
        //systems = new EndgameSystems(hardwareMap);


        //score.setLinkagePositionLogistic(constants.linkageDown, 500);
        score.setGrabberPosition(constants.open - 0.15);

        distance = hardwareMap.get(ColorRangeSensor.class, "distance");


        //Color sensor gain values
        //color.setGain(300);
        distance.setGain(180);


        //Lift Thread
        liftThread = new Thread(){
            @Override
            public void run() {
                while(opModeIsActive()){

                    //Lift up to scoring position
                    if(gamepad1.left_trigger > 0.1){
                        //score.setPower(0.2);
                        if(score.getScoringMode() != ScoringSystemV2EpicLift.ScoringMode.ULTRA && !liftBrokenMode) {
                            score.autoGoToPosition();

                            if(score.getScoringMode() == ScoringSystemV2EpicLift.ScoringMode.LOW) {
                                score.setLinkagePosition(0.71);
                            } else {
                                score.setLinkagePosition(constants.linkageScoreV2 - 0.05);
                            }
                            passive = PassivePower.EXTENDED;
                        }else if (score.getScoringMode() == ScoringSystemV2EpicLift.ScoringMode.ULTRA){
                            score.setLinkagePosition(0.15);
                        }

                    }else {
                        if(passive == PassivePower.EXTENDED){
                            score.setPowerSingular(0.23);
                        }else if(passive == PassivePower.ZERO){
                            score.setPower(0);
                        }
                    }


                    //Scoring feature
                    if(gamepad1.right_trigger > 0.1){

                        if(score.getScoringMode() != ScoringSystemV2EpicLift.ScoringMode.ULTRA) {
                            score.setGrabberPosition(constants.score);

                            try {
                                sleep(600);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }


                            linkageDown = true;

                            score.setGrabberPosition(constants.open - 0.15);


                            //Do nothing during movement phase
                            //Reset to zero and no passive power
                            score.moveToPosition(0, 0.5);
                            passive = PassivePower.ZERO;

                            //Open Grabber and reset linkage

                            //score.setLinkagePositionLogistic(constants.linkageDownV2, 300);
                            //score.setLinkagePositionLogistic(0.8, 500);
                        }else{

                            score.setGrabberPosition(constants.open - 0.15);
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
                    }else if((distance.getNormalizedColors().red > 0.80 || distance.getNormalizedColors().blue > 0.80) && autoLinkageFlag){


                        score.setGrabberPosition(constants.grabbing);

                        grabFlag = false;

                        try {
                            Thread.currentThread().sleep(150);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }

                        if(score.getScoringMode() == ScoringSystemV2EpicLift.ScoringMode.ULTRA){
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
                        score.setGrabberPosition(constants.grabbing);

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
                    if((gamepad1.left_bumper || gamepad1.dpad_up || gamepad1.dpad_down) && changeStackFlag){

                        //Raise linkage by height of a cone (max height of 5)
                        if(gamepad1.left_bumper || gamepad1.dpad_up) {
                            score.setConeStack(5);
                            score.setLinkageConeStack(false);
                            changeStackFlag = false;

                            //Lower linkage by height of a cone (min height of 1)
                        }else if(gamepad1.dpad_down){
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
                    if(!gamepad1.dpad_down && !gamepad1.dpad_up && !gamepad1.left_bumper){
                        changeStackFlag = true;
                    }


                    //Linkage up position
                    if(gamepad1.left_stick_button){
                        score.setLinkagePosition(Constants.linkageScoreV2 - 0.05);

                    }



                    //Manual open and close grabber
                    if(gamepad1.start && manualFlag){
                        if(score.getGrabberPosition() != constants.open - 0.15) {
                            score.setGrabberPosition(constants.open - 0.15);
                            try {
                                sleep(300);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                            grabFlag = true;
                        }else{
                            score.setGrabberPosition(constants.grabbing);
                            try {
                                sleep(300);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                            grabFlag = false;
                        }
                        manualFlag = false;
                    }else if(!gamepad1.start){
                        manualFlag = true;
                    }



                    //Changing scoring modes (toggle)
                    if(gamepad1.y){
                        score.setScoringMode(ScoringSystemV2EpicLift.ScoringMode.LOW);

                    }else if(gamepad1.x){
                        score.setScoringMode(ScoringSystemV2EpicLift.ScoringMode.MEDIUM);

                    }else if(gamepad1.b){
                        score.setScoringMode(ScoringSystemV2EpicLift.ScoringMode.HIGH);

                    }else if(gamepad1.a){
                        //Ultra
                        score.setScoringMode(ScoringSystemV2EpicLift.ScoringMode.ULTRA);
                    }


                    //Manual slides (dpad right and left)
                    if(gamepad1.dpad_right){
                        passive = PassivePower.MOVEMENT;
                        score.setPower(1);
                    }else if(gamepad1.dpad_left){
                        passive = PassivePower.MOVEMENT;
                        score.setPower(-0.55);
                    }else{

                        //Feedforward if slides are extended
                        if(score.isExtended() && (score.getScoringMode() == ScoringSystemV2EpicLift.ScoringMode.LOW || score.getScoringMode() == ScoringSystemV2EpicLift.ScoringMode.MEDIUM)){
                            passive = PassivePower.EXTENDED;
                        }else{
                            passive = PassivePower.ZERO;
                        }


                    }

                    if (gamepad1.ps && optionsFlag) {
                        optionsFlag = false;
                        liftBrokenMode = !liftBrokenMode;
                    }
                    if (!gamepad1.ps) {
                        optionsFlag = true;
                    }

                    if (gamepad2.dpad_up) {
                        score.setLinkagePosition(score.getLeftLinkage() + 0.001);
                    }

                    if (gamepad2.dpad_down) {
                        score.setLinkagePosition(score.getLeftLinkage() - 0.001);
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
                while(opModeIsActive()) {
                    if(linkageUp) {

                        try {
                            Thread.currentThread().sleep(100);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                        if (score.getScoringMode() == ScoringSystemV2EpicLift.ScoringMode.ULTRA) {
                            score.setLinkagePosition(0.25);
                        } else if (liftBrokenMode) {
                            score.setLinkagePosition(0.54);
                        } else {
                            score.setLinkagePosition(Constants.linkageScoreV2 - 0.05);
                        }
                        linkageUp = false;
                    }else if(linkageDown) {

                        score.setLinkagePosition(Constants.linkageUpV2);
                        try {
                            Thread.currentThread().sleep(70);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                        //score.setLinkagePositionLogistic(Constants.linkageDownV2, 300, 100);

                        //TODO: fix cone stack logic
                        score.setLinkageConeStack(true);
                        linkageDown = false;
                    }
                }
            }
        };


        waitForStart();

        //score.setLinkagePosition(constants.linkageUpV2);


        //Starting Threads
        liftThread.start();
        //capThread.start();
        linkageThread.start();

        while(opModeIsActive()){

            //N S E W Drive
            double leftStickX = gamepad1.left_stick_x;
            double leftStickY = gamepad1.left_stick_y;

            if(Math.abs(leftStickX) > Math.abs(leftStickY)){
                leftStickY = 0;

            }else if(Math.abs(leftStickY) > Math.abs(leftStickX)){
                leftStickX = 0;

            }else{
                leftStickY = 0;
                leftStickX = 0;
            }

            if (gamepad1.right_bumper) {
                drive.setPower(new Vector2D(leftStickX * constants.SPRINT_LINEAR_MODIFIER, leftStickY * constants.SPRINT_LINEAR_MODIFIER), gamepad1.right_stick_x * constants.SPRINT_ROTATIONAL_MODIFIER, false);
            } else if(score.isExtended()){
                //Slow down when slides are extended
                drive.setPower(new Vector2D(leftStickX * constants.EXTENDED_LINEAR_MODIFIER, leftStickY * constants.EXTENDED_LINEAR_MODIFIER), gamepad1.right_stick_x * constants.EXTENDED_ROTATIONAL_MODIFIER, false);
            } else{
                drive.setPower(new Vector2D(leftStickX * constants.NORMAL_LINEAR_MODIFIER, leftStickY * constants.NORMAL_LINEAR_MODIFIER), gamepad1.right_stick_x * constants.NORMAL_ROTATIONAL_MODIFIER, false);
            }


            //Telemetry

            telemetry.addData("lMotor", -1 * score.getLeftEncoderPos());
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
            telemetry.addData("passive", passive);
            telemetry.addData("coneStack", score.getConeStack());
            telemetry.addData("rip robot", liftBrokenMode);
            telemetry.update();


        }


        //Stop
        drive.simpleBrake();

        //score.setLinkagePositionLogistic(0.25, 500);
        score.setLinkagePositionLogistic(Constants.linkageDownV2, 300, 100);
        //score.setLinkagePositionLogistic(0.8, 500);




        score.setGrabberPosition(constants.open - 0.15);
    }
}
