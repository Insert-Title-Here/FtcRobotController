package org.firstinspires.ftc.teamcode.Competition.State.TeleOp.Current;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Competition.Interleagues.Common.Constants;
import org.firstinspires.ftc.teamcode.Competition.Interleagues.Common.Vector2D;
import org.firstinspires.ftc.teamcode.Competition.Interleagues.Subsystems.Used.MecDrive;
import org.firstinspires.ftc.teamcode.Competition.State.Subsystems.Current.ScoringSystemV2EpicLift;


@TeleOp (name = "KevinGodModeWheelie")
public class QuadMotorLiftWheelie extends LinearOpMode {

    ScoringSystemV2EpicLift score;
    MecDrive drive;

    double linkageToggleSpeed = 0.001;

    double wheeliePos = Constants.wheelieHigh;


    ColorRangeSensor distance;

    Servo wheelieServo;

    //PassivePower passive;

    volatile boolean autoLinkageFlag, grabFlag, shiftLinkageFlag, manualFlag, changeStackFlag, linkageUp, linkageDown, firstDpadUp, scoringPattern, changeToggle;
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
        scoringPattern = false;
        changeToggle = true;



        //Feed forward is going to be off
        //passive = PassivePower.ZERO;

        score = new ScoringSystemV2EpicLift(hardwareMap, telemetry, new ElapsedTime(), true);
        //robot = new Robot(hardwareMap);

        drive = new MecDrive(hardwareMap,false, telemetry);
        //systems = new EndgameSystems(hardwareMap);


        //score.setLinkagePositionLogistic(Constants.linkageDown, 500);
        score.setGrabberPosition(Constants.open - 0.15);

        distance = hardwareMap.get(ColorRangeSensor.class, "distance");

        wheelieServo = hardwareMap.get(Servo.class, "wheelie");


        //Color sensor gain values
        //color.setGain(300);
        distance.setGain(300);


        //Lift Thread
        liftThread = new Thread(){
            @Override
            public void run() {
                while(opModeIsActive()){

                    //Lift up to scoring position
                    if(gamepad1.left_trigger > 0.1){
                        //score.setPower(0.2);
                        if(score.getScoringMode() != ScoringSystemV2EpicLift.ScoringMode.ULTRA && !liftBrokenMode) {

                            score.commandAutoGoToPosition();

                            if(score.getScoringMode() == ScoringSystemV2EpicLift.ScoringMode.LOW) {
                                score.setLinkagePosition(0.7);
                            } else {
                                score.setLinkagePosition(Constants.linkageScoreV2 - 0.05);
                            }
                            //passive = PassivePower.EXTENDED;
                        }else if (score.getScoringMode() == ScoringSystemV2EpicLift.ScoringMode.ULTRA){
                            score.setLinkagePosition(0.15);
                        }
                        
                    }/*else {
                        if(passive == PassivePower.EXTENDED){
                            score.setPowerSingular(0.23);
                        }else if(passive == PassivePower.ZERO){
                            score.setPower(0);
                        }
                    }*/


                    //Scoring feature
                    if(gamepad1.right_trigger > 0.1){

                        if(score.getScoringMode() != ScoringSystemV2EpicLift.ScoringMode.ULTRA) {
                            score.setGrabberPosition(Constants.score);

                            try {
                                sleep(600);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }


                            linkageDown = true;

                            score.setGrabberPosition(Constants.open - 0.15);


                            //Do nothing during movement phase
                            //Reset to zero and no passive power
                            //score.moveToPosition(0, 0.5);
                            score.setLiftTarget(0);
                            //passive = PassivePower.ZERO;

                            //Open Grabber and reset linkage

                            //score.setLinkagePositionLogistic(Constants.linkageDownV2, 300);
                            //score.setLinkagePositionLogistic(0.8, 500);
                        }else{

                            score.setGrabberPosition(Constants.open - 0.15);
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

                        if(scoringPattern ){
                            if(score.getScoringMode() == ScoringSystemV2EpicLift.ScoringMode.LOW){
                                score.setScoringMode(ScoringSystemV2EpicLift.ScoringMode.MEDIUM);
                            }else if(score.getScoringMode() == ScoringSystemV2EpicLift.ScoringMode.MEDIUM){
                                score.setScoringMode(ScoringSystemV2EpicLift.ScoringMode.HIGH);
                            }else if(score.getScoringMode() == ScoringSystemV2EpicLift.ScoringMode.HIGH){
                                scoringPattern = false;
                            }
                        }


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


                        score.setGrabberPosition(Constants.grabbing);

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
                    if((gamepad1.dpad_up || gamepad1.dpad_down) && changeStackFlag){

                        //Raise linkage by height of a cone (max height of 5)
                        if(gamepad1.dpad_up) {
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
                    if(!gamepad1.dpad_down && !gamepad1.dpad_up){
                        changeStackFlag = true;
                    }


                    //Linkage up position
                    if(gamepad1.left_stick_button){
                        score.setLinkagePosition(Constants.linkageScoreV2 - 0.05);
                        wheelieServo.setPosition(wheeliePos);

                    }



                    //Manual open and close grabber
                    if(gamepad1.right_stick_button && manualFlag){
                        if(score.getGrabberPosition() != Constants.open - 0.15) {
                            score.setGrabberPosition(Constants.open - 0.15);
                            try {
                                sleep(300);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                            grabFlag = true;
                        }else{
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

                    if(!gamepad1.right_stick_button){
                        manualFlag = true;
                    }



                    //Changing scoring modes (toggle)
                    if(gamepad1.y){
                        score.setScoringMode(ScoringSystemV2EpicLift.ScoringMode.LOW);
                        wheeliePos = Constants.wheelieRetracted;

                        if(!liftBrokenMode) {
                            scoringPattern = true;
                        }

                    }else if(gamepad1.x){
                        score.setScoringMode(ScoringSystemV2EpicLift.ScoringMode.MEDIUM);
                        wheeliePos = Constants.wheelieMedium;

                    }else if(gamepad1.b){
                        score.setScoringMode(ScoringSystemV2EpicLift.ScoringMode.HIGH);
                        wheeliePos = Constants.wheelieHigh;

                    }else if(gamepad1.a){
                        //Ultra
                        score.setScoringMode(ScoringSystemV2EpicLift.ScoringMode.ULTRA);
                        wheeliePos = Constants.wheelieRetracted;
                    }


                    //Manual slides (dpad right and left)
                    if(gamepad1.dpad_right){
                        //passive = PassivePower.MOVEMENT;
                        score.setPower(1);
                        score.setLiftTarget(-1 * score.getRightEncoderPos());
                    }else if(gamepad1.dpad_left){
                        //passive = PassivePower.MOVEMENT;
                        score.setPower(-0.55);
                        score.setLiftTarget(-1 * score.getRightEncoderPos());

                    }else{
                        if(score.getLiftTarget() == 0){
                            score.newLiftPIDUpdate(0.55);
                            telemetry.addData("stuff", "slow");

                        }else {
                            score.newLiftPIDUpdate(1);
                            telemetry.addData("stuff", "fast");

                        }

                    }

                    telemetry.addData("target", score.getLiftTarget());

                    telemetry.update();



                    /*else{

                        //Feedforward if slides are extended
                        if(score.isExtended() && (score.getScoringMode() == ScoringSystemV2EpicLift.ScoringMode.LOW || score.getScoringMode() == ScoringSystemV2EpicLift.ScoringMode.MEDIUM)){
                            passive = PassivePower.EXTENDED;
                        }else{
                            passive = PassivePower.ZERO;
                        }


                    }*/

                    if (gamepad1.left_bumper && optionsFlag) {
                        optionsFlag = false;
                        liftBrokenMode = !liftBrokenMode;

                        if(liftBrokenMode){
                            gamepad1.rumble(1500);
                        }else{
                            gamepad1.rumble(200);
                        }
                    }
                    if (!gamepad1.left_bumper) {
                        optionsFlag = true;
                    }

                   /* if (gamepad2.dpad_up) {
                        score.setLinkagePosition(score.getLeftLinkage() + 0.001);
                    }else if (gamepad2.dpad_down) {
                        score.setLinkagePosition(score.getLeftLinkage() - 0.001);
                    }else if(gamepad2.dpad_left){
                        score.setLinkagePosition(score.getLeftLinkage() - 0.002);
                    }else if(gamepad2.dpad_right){
                        score.setLinkagePosition(score.getLeftLinkage() + 0.002);
                    }*/


                    if((gamepad2.dpad_up || gamepad2.dpad_down) && changeToggle){
                        if(gamepad2.dpad_up){
                            score.setLinkagePosition(score.getLeftLinkage() + 0.025);

                        }else{
                            score.setLinkagePosition(score.getLeftLinkage() - 0.025);


                        }

                        changeToggle = false;


                    }

                    if(!gamepad2.dpad_up && !gamepad2.dpad_down){
                        changeToggle = true;
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

                        wheelieServo.setPosition(wheeliePos);

                        linkageUp = false;
                    }else if(linkageDown) {

                        score.setLinkagePosition(Constants.linkageUpV2);
                        try {
                            Thread.currentThread().sleep(70);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                        //score.setLinkagePositionLogistic(Constants.linkageDownV2, 300, 100);

                        //wheelieServo.setPosition(Constants.wheelieRetracted);

                        //TODO: fix cone stack logic
                        score.setLinkageConeStack(true);
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

            /*if (gamepad1.right_bumper) {
                drive.setPower(new Vector2D(leftStickX * Constants.SPRINT_LINEAR_MODIFIER, leftStickY * Constants.SPRINT_LINEAR_MODIFIER), gamepad1.right_stick_x * Constants.SPRINT_ROTATIONAL_MODIFIER, false);
            } else*/
            if(score.isExtended()){
                //Slow down when slides are extended
                drive.setPower(new Vector2D(leftStickX * Constants.EXTENDED_LINEAR_MODIFIER, leftStickY * Constants.EXTENDED_LINEAR_MODIFIER), gamepad1.right_stick_x * Constants.EXTENDED_ROTATIONAL_MODIFIER, false);
            } else{
                drive.setPower(new Vector2D(leftStickX/* * Constants.NORMAL_LINEAR_MODIFIER*/, leftStickY/* * Constants.NORMAL_LINEAR_MODIFIER*/), gamepad1.right_stick_x * Constants.NORMAL_ROTATIONAL_MODIFIER, false);
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




        score.setGrabberPosition(Constants.open - 0.15);
    }
}
