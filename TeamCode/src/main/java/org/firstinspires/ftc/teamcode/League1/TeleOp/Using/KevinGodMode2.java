package org.firstinspires.ftc.teamcode.League1.TeleOp.Using;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.League1.Common.Constants;
import org.firstinspires.ftc.teamcode.League1.Common.Robot;
import org.firstinspires.ftc.teamcode.League1.Common.Vector2D;
import org.firstinspires.ftc.teamcode.League1.Subsystems.EndgameSystems;
import org.firstinspires.ftc.teamcode.League1.Subsystems.MecDrive;
import org.firstinspires.ftc.teamcode.League1.Subsystems.ScoringSystem;
import org.firstinspires.ftc.teamcode.League1.Subsystems.ScoringSystem2;

import java.util.concurrent.atomic.AtomicBoolean;


//TODO: figure out bulk read

@Disabled
@TeleOp (name = "KevinGodMode")
public class KevinGodMode2 extends LinearOpMode {

    Constants constants = new Constants();
    ScoringSystem2 score;
    MecDrive drive;
    EndgameSystems systems;

    ColorRangeSensor distance, color;

    PassivePower passive;

    boolean previousLeft, previousRight, previousUp, previousDown;
    volatile boolean autoLinkageFlag, grabFlag, shiftLinkageFlag, manualFlag, changeStackFlag, linkageUp, linkageDown;

    Thread liftThread/*, capThread*/, linkageThread;

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

        score = new ScoringSystem2(hardwareMap, constants);
        //robot = new Robot(hardwareMap);
        drive = new MecDrive(hardwareMap,false, telemetry);
        systems = new EndgameSystems(hardwareMap);


        //score.setLinkagePositionLogistic(constants.linkageDown, 500);
        score.setGrabberPosition(constants.open - 0.11);

        distance = hardwareMap.get(ColorRangeSensor.class, "distance");
        //color = hardwareMap.get(ColorRangeSensor.class, "color");

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
                        score.setLinkagePosition(constants.linkageScore);
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
                        score.setGrabberPosition(constants.score);

                        //Low height logic (need to lift slides up a bit before bringing linkage back for clearance)
                        if(score.getScoringMode() == ScoringSystem2.ScoringMode.LOW && score.isExtended()) {
                            try {
                                sleep(500);
                            } catch (InterruptedException e) {

                            }
                            passive = PassivePower.ZERO;
                            score.moveToPosition(constants.lowOperation, 1);
                            passive = PassivePower.EXTENDED;

                        }

                        try {
                            sleep(600);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }



                        //score.setLinkagePosition(constants.linkageUp);
                        score.lowerConeStack();
                        linkageDown = true;




                        //Do nothing during movement phase
                        passive = PassivePower.MOVEMENT;

                        //Reset to zero and no passive power
                        score.moveToPosition(0, 0.5);
                        passive = PassivePower.ZERO;

                        //Open Grabber and reset linkage
                        score.setGrabberPosition(constants.open - 0.11);
                        //score.setLinkagePositionLogistic(constants.linkageDown, 300);



                        //Resetting flags
                        autoLinkageFlag = true;
                        grabFlag = true;

                        //Not extended anymore
                        score.setExtended(false);

                        //Automated Grab
                    }else if((distance.getDistance(DistanceUnit.CM) < 3.5) && grabFlag) {
                        score.setGrabberPosition(constants.grabbing);

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
                            score.setLinkagePosition(constants.linkageScore);
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
                        score.setLinkagePositionLogistic(constants.linkageUp, 500);

                    }



                    //Manual open and close grabber
                    if(gamepad1.start && manualFlag){
                        if(score.getGrabberPosition() != constants.open - 0.11) {
                            score.setGrabberPosition(constants.open - 0.11);
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
                            Thread.sleep(100);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                        score.setLinkagePositionLogistic(constants.linkageUp, 0, 50);
                        linkageUp = false;
                    }else if(linkageDown) {

                        score.setLinkagePosition(Constants.linkageUp);

                        score.setLinkageConeStack(true);
                        linkageDown = false;
                    }
                }
            }
        };

        //TODO: might need to change this

        waitForStart();
        score.setLinkagePosition(constants.linkageUp);


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
        score.setLinkagePositionLogistic(constants.linkageDown, 500);
        score.setGrabberPosition(constants.open - 0.11);
    }
}
