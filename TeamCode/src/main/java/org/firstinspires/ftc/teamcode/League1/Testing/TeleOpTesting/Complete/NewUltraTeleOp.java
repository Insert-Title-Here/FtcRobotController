package org.firstinspires.ftc.teamcode.League1.Testing.TeleOpTesting.Complete;

import com.qualcomm.hardware.lynx.LynxModule;
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

@TeleOp
public class NewUltraTeleOp extends LinearOpMode {


    private final double NORMAL_LINEAR_MODIFIER = 0.75;
    private final double NORMAL_ROTATIONAL_MODIFIER = 0.5;
    private final double EXTENDED_LINEAR_MODIFIER = 0.5;
    private final double EXTENDED_ROTATIONAL_MODIFIER = 0.3;
    private final double SPRINT_LINEAR_MODIFIER = 1;
    private final double SPRINT_ROTATIONAL_MODIFIER = 1;


    Constants constants = new Constants();
    ScoringSystem2 score;
    MecDrive drive;
    //Robot robot;
    ColorRangeSensor distance, color;
    EndgameSystems systems;

    PassivePower passive;

    boolean previousLeft, previousRight, previousUp, previousDown;
    volatile boolean autoLinkageFlag, grabFlag, shiftLinkageFlag, manualFlag;

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
        shiftLinkageFlag = true;
        manualFlag = true;

        //Initialization
        passive = PassivePower.ZERO;

        score = new ScoringSystem2(hardwareMap, constants);
        //robot = new Robot(hardwareMap);
        drive = new MecDrive(hardwareMap,false, telemetry);
        systems = new EndgameSystems(hardwareMap);


        score.setLinkagePosition(0.1);
        score.setGrabberPosition(0.75);

        distance = hardwareMap.get(ColorRangeSensor.class, "distance");
        color = hardwareMap.get(ColorRangeSensor.class, "color");

        color.setGain(300);
        distance.setGain(300);


        //Lift Thread
        //TODO: see if linkage here works
        liftThread = new Thread(){
            @Override
            public void run() {
                while(opModeIsActive()){

                    if(gamepad1.left_trigger > 0.1){
                        score.autoGoToPosition();
                        score.setLinkagePosition(constants.linkageScore);
                        passive = PassivePower.EXTENDED;

                    }else {
                        if(passive == PassivePower.EXTENDED){
                            score.setPower(0.2);
                        }else if(passive == PassivePower.DOWN){

                        }else if(passive == PassivePower.ZERO){
                            score.setPower(0);
                        }
                    }


                    if(gamepad1.right_trigger > 0.1){
                        score.setGrabberPosition(constants.openAuto);


                        try {
                            sleep(300);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }

                        score.setLinkagePosition(0.7);

                        try {
                            sleep(300);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }

                        passive = PassivePower.DOWN;
                        score.moveToPosition(0, 0.5);
                        passive = PassivePower.ZERO;
                        score.setLinkagePosition(0.1);

                        autoLinkageFlag = true;
                        grabFlag = true;
                        score.setExtended(false);

                        //Automated Grab and Score
                    }else if((distance.getDistance(DistanceUnit.CM) < 6.5) && grabFlag) {
                        score.setGrabberPosition(constants.grabbing);

                        grabFlag = false;
                        try {
                            sleep(200);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }

                    }

                    //TODO: fix this logic
                    if((distance.getNormalizedColors().red > 0.85 || distance.getNormalizedColors().blue > 0.85) && autoLinkageFlag){
                        score.setLinkagePosition(0.7);
                        autoLinkageFlag = false;
                        //telemetry.addData("Is this trippin", "yes");

                        if(score.getScoringMode() == ScoringSystem2.ScoringMode.ULTRA){
                            score.autoGoToPosition();
                            score.setLinkagePosition(constants.linkageScore);
                            passive = PassivePower.EXTENDED;


                        }
                    }

                    //Auto cone heights
                    if(gamepad1.left_bumper && shiftLinkageFlag){
                        score.shiftLinkagePosition();
                        shiftLinkageFlag = false;
                    }else{
                        shiftLinkageFlag = true;
                    }



                    //Manual open and close grabber
                    if(gamepad1.start && manualFlag){
                        if(score.getGrabberPosition() != constants.openAuto) {
                            score.setGrabberPosition(constants.openAuto);
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

            if (gamepad1.right_bumper) {
                drive.setPower(new Vector2D(leftStickX * SPRINT_LINEAR_MODIFIER, leftStickY * SPRINT_LINEAR_MODIFIER), gamepad1.right_stick_x * SPRINT_ROTATIONAL_MODIFIER, false);
            } else if(score.isExtended()){
                //Slow down when slides are extended
                drive.setPower(new Vector2D(leftStickX * EXTENDED_LINEAR_MODIFIER, leftStickY * EXTENDED_LINEAR_MODIFIER), gamepad1.right_stick_x * EXTENDED_ROTATIONAL_MODIFIER, false);
            } else{
                drive.setPower(new Vector2D(leftStickX * NORMAL_LINEAR_MODIFIER, leftStickY * NORMAL_LINEAR_MODIFIER), gamepad1.right_stick_x * NORMAL_ROTATIONAL_MODIFIER, false);
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
            /*telemetry.addData("colorRed: ", color.getNormalizedColors().red);
            telemetry.addData("colorBlue: ", color.getNormalizedColors().blue);*/
            telemetry.addData("rightServoTarget", score.getRightLinkage());
            telemetry.addData("leftServoTarget", score.getLeftLinkage());
            telemetry.addData("passive", passive);
            telemetry.update();

        }
        //Stop
        drive.setPower(0, 0, 0, 0);
        score.setLinkagePosition(0.2);
        sleep(500);
        score.setLinkagePosition(0.1);
        score.setGrabberPosition(constants.openAuto);
    }
}
