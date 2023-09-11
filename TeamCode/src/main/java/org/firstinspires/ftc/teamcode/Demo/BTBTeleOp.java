package org.firstinspires.ftc.teamcode.Demo;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Competition.Interleagues.Common.Constants;
import org.firstinspires.ftc.teamcode.Competition.Interleagues.Subsystems.Used.EndgameSystems;
import org.firstinspires.ftc.teamcode.Competition.Interleagues.Subsystems.Used.MecDrive;
import org.firstinspires.ftc.teamcode.Competition.Interleagues.Subsystems.Used.ScoringSystem2;


//TODO: figure out bulk read

@Disabled
@TeleOp (name = "BTB TeleOp")
public class BTBTeleOp extends LinearOpMode {

    ////Constants //constants = newConstants();
    ScoringSystem2 score;
    MecDrive drive;
    EndgameSystems systems;

    ColorRangeSensor distance, color;

    PassivePower passive;
    Servo cameraServo;

    boolean previousLeft, previousRight, previousUp, previousDown, linkageUp;
    volatile boolean autoLinkageFlag, grabFlag, shiftLinkageFlag, manualFlag, changeStackFlag;

    Thread liftThread, capThread, linkageThread;

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

        //Feed forward is going to be off
        passive = PassivePower.ZERO;

        score = new ScoringSystem2(hardwareMap);
        //robot = new Robot(hardwareMap);
        drive = new MecDrive(hardwareMap,false, telemetry);
        systems = new EndgameSystems(hardwareMap);


        //score.setLinkagePositionLogistic(Constants.linkageDown, 500);
        score.setGrabberPosition(Constants.open);

        distance = hardwareMap.get(ColorRangeSensor.class, "distance");
        //color = hardwareMap.get(ColorRangeSensor.class, "color");
        cameraServo = hardwareMap.get(Servo.class, "camera");

        //Color sensor gain values
        color.setGain(300);
        distance.setGain(300);


        //Lift Thread
        liftThread = new Thread(){
            @Override
            public void run() {
                while(opModeIsActive()){

                    //Lift up to scoring position
                    if(gamepad1.left_trigger > 0.1){
                        score.autoGoToPosition();
                        score.setPower(0.2);
                        score.setLinkagePosition(Constants.linkageScore);
                        passive = PassivePower.EXTENDED;

                    }else {
                        if(passive == PassivePower.EXTENDED){
                            score.setPower(0.23);
                        }else if(passive == PassivePower.ZERO){
                            score.setPower(0);
                        }
                    }


                    //Scoring feature
                    if(gamepad1.right_trigger > 0.1){
                        score.setGrabberPosition(Constants.score);

                        //Low height logic (need to lift slides up a bit before bringing linkage back for clearance)
                        if(score.getScoringMode() == ScoringSystem2.ScoringMode.LOW && score.isExtended()) {
                            try {
                                sleep(500);
                            } catch (InterruptedException e) {

                            }
                            passive = PassivePower.ZERO;
                            score.moveToPosition(Constants.lowOperation, 1);
                            passive = PassivePower.EXTENDED;

                        }

                        try {
                            sleep(600);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }



                        score.setLinkagePosition(Constants.linkageUp);


                        try {
                            sleep(300);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }



                        //Do nothing during movement phase
                        passive = PassivePower.MOVEMENT;

                        //Reset to zero and no passive power
                        score.moveToPosition(0, 0.5);
                        passive = PassivePower.ZERO;

                        //Open Grabber and reset linkage
                        score.setGrabberPosition(Constants.open);
                        //score.setLinkagePositionLogistic(Constants.linkageDown, 300);

                        score.lowerConeStack();
                        score.setLinkageConeStack(true);

                        //Resetting flags
                        autoLinkageFlag = true;
                        grabFlag = true;

                        //Not extended anymore
                        score.setExtended(false);

                        //Automated Grab
                    }else if((distance.getDistance(DistanceUnit.CM) < 6.5) && grabFlag) {
                        score.setGrabberPosition(Constants.grabbing);

                        grabFlag = false;
                        try {
                            sleep(200);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }

                    }


                    //TODO: see if need to fix this logic
                    //Auto linkage up logic after sensing a cone
                    if((distance.getNormalizedColors().red > 0.85 || distance.getNormalizedColors().blue > 0.85) && autoLinkageFlag){
                        linkageUp = true;
                        autoLinkageFlag = false;

                        //Goes up automatically if in ultra mode
                        if(score.getScoringMode() == ScoringSystem2.ScoringMode.ULTRA){
                            score.autoGoToPosition();
                            score.setPower(0.2);
                            try {
                                Thread.sleep(100);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                            score.setLinkagePosition(Constants.linkageScore);
                            passive = PassivePower.EXTENDED;


                        }
                    }

                    //Linkage stack cone heights with dpad up and down
                    if((gamepad1.left_bumper || gamepad1.dpad_up || gamepad1.dpad_down) && changeStackFlag){

                        //Raise linkage by height of a cone (max height of 5)
                        if(gamepad1.left_bumper || gamepad1.dpad_up) {
                            score.raiseConeStack();
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


                    //Reset linkage position
                    if(gamepad1.left_stick_button){
                        score.setLinkagePositionLogistic(Constants.linkageUp, 500);

                    }



                    //Manual open and close grabber
                    if(gamepad1.start && manualFlag){
                        if(score.getGrabberPosition() != Constants.open) {
                            score.setGrabberPosition(Constants.open);
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
                    }else if(!gamepad1.start){
                        manualFlag = true;
                    }



                    //Changing scoring modes (toggle)
                    if(gamepad1.y){
                        score.setScoringMode(ScoringSystem2.ScoringMode.LOW);

                    }else if(gamepad1.x){
                        score.setScoringMode(ScoringSystem2.ScoringMode.MEDIUM);

                    }else if(gamepad1.b){
                        score.setScoringMode(ScoringSystem2.ScoringMode.HIGH);

                    }else if(gamepad1.a){
                        //Ultra
                        score.setScoringMode(ScoringSystem2.ScoringMode.ULTRA);
                    }


                    //Manual slides (dpad right and left)
                    if(gamepad1.dpad_right){
                        passive = PassivePower.MOVEMENT;
                        score.setPower(0.7);
                    }else if(gamepad1.dpad_left){
                        passive = PassivePower.MOVEMENT;
                        score.setPower(-0.3);
                    }else{

                        //Feedforward if slides are extended
                        if(score.isExtended() == true){
                            passive = PassivePower.EXTENDED;
                        }else{
                            passive = PassivePower.ZERO;
                        }
                    }


                }

            }
        };



        //Logistic Linkage thread
        linkageThread = new Thread() {

            @Override
            public void run() {
                while(opModeIsActive()) {
                    if(linkageUp) {

                        try {
                            Thread.sleep(100);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                        score.setLinkagePositionLogistic(Constants.linkageUp, 0, 50);
                        linkageUp = false;
                    }
                }
            }
        };

        //TODO: might need to change this

        waitForStart();
        score.setLinkagePosition(Constants.linkageUp);


        //Starting Threads
        liftThread.start();
        linkageThread.start();

        while(opModeIsActive()){

            if (gamepad2.dpad_left) {
                cameraServo.setPosition(Constants.pole);
            } else if (gamepad2.dpad_up) {
                cameraServo.setPosition(Constants.cone);
            } else if(gamepad2.dpad_down) {
                cameraServo.setPosition(Constants.sleeve);
            }

            /*
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
                drive.setPower(new Vector2D(leftStickX * Constants.SPRINT_LINEAR_MODIFIER, leftStickY * Constants.SPRINT_LINEAR_MODIFIER), gamepad1.right_stick_x * Constants.SPRINT_ROTATIONAL_MODIFIER, false);
            } else if(score.isExtended()){
                //Slow down when slides are extended
                drive.setPower(new Vector2D(leftStickX * Constants.EXTENDED_LINEAR_MODIFIER, leftStickY * Constants.EXTENDED_LINEAR_MODIFIER), gamepad1.right_stick_x * Constants.EXTENDED_ROTATIONAL_MODIFIER, false);
            } else{
                drive.setPower(new Vector2D(leftStickX * Constants.NORMAL_LINEAR_MODIFIER, leftStickY * Constants.NORMAL_LINEAR_MODIFIER), gamepad1.right_stick_x * Constants.NORMAL_ROTATIONAL_MODIFIER, false);
            }

             */




            //Telemetry
            telemetry.addData("lMotor", -1 * score.getLeftEncoderPos());
            telemetry.addData("rMotor", score.getRightEncoderPos());
            telemetry.addData("distance: ", distance.getDistance(DistanceUnit.CM));
            telemetry.addData("distanceRed", distance.getNormalizedColors().red);
            telemetry.addData("distanceBlue", distance.getNormalizedColors().blue);
            telemetry.addData("colorRed", color.getNormalizedColors().red);
            telemetry.addData("colorBlue", color.getNormalizedColors().blue);
            telemetry.addData("autoLinkageFlag", autoLinkageFlag);
            telemetry.addData("grabbingFlag", grabFlag);
            telemetry.addData("manualFlag", manualFlag);
            telemetry.addData("shiftLinkageFlag", shiftLinkageFlag);
            telemetry.addData("extended", score.isExtended());
            /*telemetry.addData("colorRed: ", color.getNormalizedColors().red);
            telemetry.addData("colorBlue: ", color.getNormalizedColors().blue);*/
            telemetry.addData("rightServoTarget", score.getRightLinkage());
            telemetry.addData("leftServoTarget", score.getLeftLinkage());
            telemetry.addData("passive", passive);
            telemetry.addData("coneStack", score.getConeStack());
            telemetry.update();

        }


        //Stop
        drive.setPower(0, 0, 0, 0);
        score.setLinkagePositionLogistic(0.25, 500);
        score.setLinkagePositionLogistic(Constants.linkageDown, 500);
        score.setGrabberPosition(Constants.open);
    }
}
