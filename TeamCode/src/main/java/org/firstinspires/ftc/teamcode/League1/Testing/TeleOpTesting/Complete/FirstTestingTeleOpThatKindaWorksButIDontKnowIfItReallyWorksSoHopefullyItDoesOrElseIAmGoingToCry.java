package org.firstinspires.ftc.teamcode.League1.Testing.TeleOpTesting.Complete;

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
@TeleOp
public class FirstTestingTeleOpThatKindaWorksButIDontKnowIfItReallyWorksSoHopefullyItDoesOrElseIAmGoingToCry extends LinearOpMode {


    private final double NORMAL_LINEAR_MODIFIER = 0.75;
    private final double NORMAL_ROTATIONAL_MODIFIER = 0.5;
    private final double SPRINT_LINEAR_MODIFIER = 1;
    private final double SPRINT_ROTATIONAL_MODIFIER = 1;


    ////Constants //constants = newConstants();
    ScoringSystem2 score;
    MecDrive drive;
    //Robot robot;
    ColorRangeSensor distance, color;

    AtomicBoolean goDown;

    EndgameSystems systems;

    PassivePower passive;

    boolean previousLeft, previousRight, previousUp, previousDown;
    volatile boolean autoLinkageFlag, grabFlag;

    Thread liftThread, capThread;


    public enum PassivePower{
        EXTENDED,
        DOWN,
        ZERO,

    }


    @Override
    public void runOpMode() throws InterruptedException {
        autoLinkageFlag = true;
        grabFlag = true;

        //Initialization
        passive = PassivePower.ZERO;
        goDown = new AtomicBoolean(false);

        score = new ScoringSystem2(hardwareMap);
        //robot = new Robot(hardwareMap);
        drive = new MecDrive(hardwareMap,false, telemetry);
        systems = new EndgameSystems(hardwareMap);


        score.setLinkagePosition(0.95);
        score.setGrabberPosition(0.75);

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
                        score.setPower(-gamepad1.left_trigger / 3);
                    }else if(gamepad1.right_bumper){
                        score.moveToPosition(850, 1);

                        score.setLinkagePosition(0.05);
                        passive = PassivePower.EXTENDED;
                    }else if(gamepad1.y){
                        //TODO: Make this medium height
                        score.moveToPosition(500, 1);
                        score.setLinkagePosition(0.05);

                        passive = PassivePower.EXTENDED;
                    }else if(gamepad1.b){
                        //TODO: Make this low height
                        score.moveToPosition(190, 1);
                        score.setLinkagePosition(0.05);

                        passive = PassivePower.EXTENDED;
                    }else {
                        if(passive == PassivePower.EXTENDED){
                            score.setPower(0.2);
                        }else if(passive == PassivePower.DOWN){

                        }else if(passive == PassivePower.ZERO){
                            score.setPower(0);
                        }
                    }


                    //
                    if(goDown.get()) {
                        score.setGrabberPosition(Constants.open);

                        try {
                            sleep(300);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }

                        score.setLinkagePosition(0.35);

                        try {
                            sleep(2000);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }

                        passive = PassivePower.DOWN;
                        score.moveToPosition(0, 0.5);
                        passive = PassivePower.ZERO;
                        score.setLinkagePosition(0.95);

                        autoLinkageFlag = true;
                        grabFlag = true;

                        goDown.set(false);
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
                    systems.setXCapstoneRotatePower(gamepad2.left_stick_x);

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

            if (gamepad1.left_bumper) {
                drive.setPower(new Vector2D(leftStickX * SPRINT_LINEAR_MODIFIER, leftStickY * SPRINT_LINEAR_MODIFIER), gamepad1.right_stick_x * SPRINT_ROTATIONAL_MODIFIER, false);
            } else {
                drive.setPower(new Vector2D(leftStickX * NORMAL_LINEAR_MODIFIER, leftStickY * NORMAL_LINEAR_MODIFIER), gamepad1.right_stick_x * NORMAL_ROTATIONAL_MODIFIER, false);
            }

            if(gamepad1.x){
                score.setGrabberPosition(Constants.open);
                grabFlag = true;
            }


            if(gamepad1.start){
                score.setLinkagePosition(0.9);
                sleep(200);
                score.setLinkagePosition(0.95);

                autoLinkageFlag = true;
            }

            //Automated Grab and Score
            if(gamepad1.right_trigger > 0.1){
                goDown.set(true);

            }else if((distance.getDistance(DistanceUnit.CM) < 6.5) && grabFlag) {
                score.setGrabberPosition(Constants.grabbing);

                grabFlag = false;
                sleep(200);

            }


            if((distance.getNormalizedColors().red > 0.85 || distance.getNormalizedColors().blue > 0.85) && autoLinkageFlag){
                score.setLinkagePosition(0.35);
                autoLinkageFlag = false;
            }


            //Auto cone heights
            //TODO: tune this
            if(gamepad1.dpad_up){
                score.setLinkagePosition(0.84);

            }else if(gamepad1.dpad_left){
                score.setLinkagePosition(0.86);

            }else if(gamepad1.dpad_left){
                score.setLinkagePosition(0.88);

            }else if(gamepad1.dpad_left){
                score.setLinkagePosition(0.9);

            }

            //Telemetry
            telemetry.addData("lMotor", -1 * score.getLeftEncoderPos());
            telemetry.addData("rMotor", score.getRightEncoderPos());
            telemetry.addData("distance: ", distance.getDistance(DistanceUnit.CM));
            telemetry.addData("distanceRed", distance.getNormalizedColors().red);
            telemetry.addData("distanceBlue", distance.getNormalizedColors().blue);
            telemetry.addData("autoLinkageFlag", autoLinkageFlag);
            telemetry.addData("grabbingFlag", grabFlag);
            /*telemetry.addData("colorRed: ", color.getNormalizedColors().red);
            telemetry.addData("colorBlue: ", color.getNormalizedColors().blue);*/
            telemetry.addData("rightServoTarget", score.getRightLinkage());
            telemetry.addData("leftServoTarget", score.getLeftLinkage());
            telemetry.addData("passive", passive);
            telemetry.update();

        }
        //Stop
        drive.setPower(0, 0, 0, 0);
        score.setLinkagePosition(0.9);
        sleep(500);
        score.setLinkagePosition(0.95);
        score.setGrabberPosition(Constants.open);
    }
}
