package org.firstinspires.ftc.teamcode.Competition.Interleagues.TeleOp.Older;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Competition.Interleagues.Common.Constants;
import org.firstinspires.ftc.teamcode.Competition.Interleagues.Common.Vector2D;
import org.firstinspires.ftc.teamcode.Competition.Interleagues.Subsystems.Used.EndgameSystems;
import org.firstinspires.ftc.teamcode.Competition.Interleagues.Subsystems.Used.MecDrive;
import org.firstinspires.ftc.teamcode.Competition.Interleagues.Subsystems.Used.ScoringSystem2;


//TODO: figure out bulk read
@Disabled
@TeleOp (name = "Nothing")
public class NewUltraTeleOp extends LinearOpMode {

    ////Constants //constants = newConstants();
    ScoringSystem2 score;
    MecDrive drive;
    ColorRangeSensor distance, color;
    EndgameSystems systems;

    PassivePower passive;

    boolean previousLeft, previousRight, previousUp, previousDown;
    volatile boolean autoLinkageFlag, grabFlag, shiftLinkageFlag, manualFlag, changeStackFlag;

    Thread liftThread, capThread;

    public enum PassivePower{
        EXTENDED,
        MOVEMENT,
        ZERO,
    }


    @Override
    public void runOpMode() throws InterruptedException {
        autoLinkageFlag = true;
        grabFlag = true;
        shiftLinkageFlag = true;
        manualFlag = true;

        //Initialization
        passive = PassivePower.ZERO;

        score = new ScoringSystem2(hardwareMap);
        //robot = new Robot(hardwareMap);
        drive = new MecDrive(hardwareMap,false, telemetry);
        systems = new EndgameSystems(hardwareMap);


        score.setLinkagePosition(Constants.linkageDown);
        score.setGrabberPosition(Constants.open);

        distance = hardwareMap.get(ColorRangeSensor.class, "distance");
        //color = hardwareMap.get(ColorRangeSensor.class, "color");

        color.setGain(300);
        distance.setGain(300);


        //Lift Thread
        liftThread = new Thread(){
            @Override
            public void run() {
                while(opModeIsActive()){

                    if(gamepad1.left_trigger > 0.1){
                        score.autoGoToPosition();
                        score.setLinkagePosition(Constants.linkageScore);
                        passive = PassivePower.EXTENDED;

                    }else {
                        if(passive == PassivePower.EXTENDED){
                            score.setPower(0.2);
                        }else if(passive == PassivePower.MOVEMENT){

                        }else if(passive == PassivePower.ZERO){
                            score.setPower(0);
                        }
                    }


                    if(gamepad1.right_trigger > 0.1){
                        score.setGrabberPosition(Constants.open);

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
                            sleep(300);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }

                        score.setLinkagePosition(Constants.linkageUp);

                        try {
                            sleep(300);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }

                        passive = PassivePower.MOVEMENT;
                        score.moveToPosition(0, 0.5);
                        passive = PassivePower.ZERO;
                        score.setLinkagePosition(Constants.linkageDown);

                        autoLinkageFlag = true;
                        grabFlag = true;
                        score.setExtended(false);

                        //Automated Grab and Score
                    }else if((distance.getDistance(DistanceUnit.CM) < 6.5) && grabFlag) {
                        score.setGrabberPosition(Constants.grabbing);

                        grabFlag = false;
                        try {
                            sleep(200);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }

                    }

                    //TODO: fix this logic
                    if((distance.getNormalizedColors().red > 0.85 || distance.getNormalizedColors().blue > 0.85) && autoLinkageFlag){
                        score.setLinkagePosition(Constants.linkageUp);
                        autoLinkageFlag = false;
                        //telemetry.addData("Is this trippin", "yes");

                        if(score.getScoringMode() == ScoringSystem2.ScoringMode.ULTRA){
                            score.autoGoToPosition();
                            score.setLinkagePosition(Constants.linkageScore);
                            passive = PassivePower.EXTENDED;


                        }
                    }

                    /*//Auto cone heights
                    if(gamepad1.left_bumper && shiftLinkageFlag){
                        score.shiftLinkagePosition();
                        shiftLinkageFlag = false;
                    }else{
                        shiftLinkageFlag = true;
                    }*/

                    //TODO: test this
                    //Linkage stack cone heights with dpad up and down
                    if((gamepad1.dpad_up || gamepad1.dpad_down) && changeStackFlag){
                        if(gamepad1.dpad_up) {
                            score.raiseConeStack();
                            score.setLinkageConeStack(false);
                            changeStackFlag = false;
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

                    }else{
                        changeStackFlag = true;
                    }


                    //Reset linkage position
                    if(gamepad1.left_stick_button){
                        score.setLinkagePosition(Constants.linkageDown);

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
                    }else{
                        manualFlag = true;
                    }





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
                        if(score.isExtended() == true){
                            passive = PassivePower.EXTENDED;
                        }else{
                            passive = PassivePower.ZERO;
                        }
                    }


                }

            }
        };

        //CapThread
        capThread = new Thread(){


            @Override
            public void run() {
                while(opModeIsActive()){

                    if(gamepad2.right_trigger > 0 || gamepad2.left_trigger > 0) {
                        double val = gamepad2.right_trigger - gamepad2.left_trigger;
                        systems.setCapstoneExtensionPower(val);
                    } else if (gamepad2.right_bumper) {
                        systems.setCapstoneExtensionPower(0.5);
                    } else if (gamepad2.left_bumper) {
                        systems.setCapstoneExtensionPower(-0.5);
                    } else{
                        systems.setCapstoneExtensionPower(0);
                    }



                    double yPos = systems.getYCapPosition();
                    systems.setXCapstoneRotatePower(gamepad2.right_stick_x);

                    systems.setYCapPosition(yPos - systems.map(gamepad2.right_stick_y, -1, 1, -0.0010, 0.0010));

                    if (gamepad2.x) {
                        systems.zeroCap();
                    } else if (gamepad2.dpad_right && previousRight != gamepad2.dpad_right) {
                        systems.setXCapSpeedDivisor(7);
                    } else if (gamepad2.dpad_left && previousLeft != gamepad2.dpad_left) {
                        systems.setXCapSpeedDivisor(4);
                    }

                    previousLeft = gamepad2.dpad_left;
                    previousRight = gamepad2.dpad_right;
                    previousUp = gamepad2.dpad_up;
                    previousDown = gamepad2.dpad_down;
                }

                systems.zeroCap();
                systems.setXCapstoneRotatePower(0);
                systems.setCapstoneExtensionPower(0);


            }
        };

        waitForStart();

        liftThread.start();
        capThread.start();

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
                drive.setPower(new Vector2D(leftStickX * Constants.SPRINT_LINEAR_MODIFIER, leftStickY * Constants.SPRINT_LINEAR_MODIFIER), gamepad1.right_stick_x * Constants.SPRINT_ROTATIONAL_MODIFIER, false);
            } else if(score.isExtended()){
                //Slow down when slides are extended
                drive.setPower(new Vector2D(leftStickX * Constants.EXTENDED_LINEAR_MODIFIER, leftStickY * Constants.EXTENDED_LINEAR_MODIFIER), gamepad1.right_stick_x * Constants.EXTENDED_ROTATIONAL_MODIFIER, false);
            } else{
                drive.setPower(new Vector2D(leftStickX * Constants.NORMAL_LINEAR_MODIFIER, leftStickY * Constants.NORMAL_LINEAR_MODIFIER), gamepad1.right_stick_x * Constants.NORMAL_ROTATIONAL_MODIFIER, false);
            }


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
        score.setLinkagePosition(0.25);
        sleep(500);
        score.setLinkagePosition(Constants.linkageDown);
        score.setGrabberPosition(Constants.open);
    }
}
