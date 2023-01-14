package org.firstinspires.ftc.teamcode.V2.TeleOp;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.util.Timing;
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

import java.util.Timer;
import java.util.concurrent.TimeUnit;


//TODO: cant drive when lift is going down

@TeleOp(name = "New TeleOp (Dont use yet)")
public class CommandTeleOpTesting extends LinearOpMode {

    ScoringSystemV2EpicLift score;
    MecDrive drive;

    double startTime, currentTime;
    ElapsedTime time;
    int liftTarget;


    ColorRangeSensor distance;


    volatile boolean autoLinkageFlag, grabFlag, shiftLinkageFlag, manualFlag, changeStackFlag, linkageUp, linkageDown, firstDpadUp, scoringPattern;
    volatile boolean liftBrokenMode = false;
    volatile boolean optionsFlag = true;

    Thread liftThread, linkageThread;

    //Enums for feed forward
    public enum PassivePower {
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
        liftTarget = 0;


        //Feed forward is going to be off

        score = new ScoringSystemV2EpicLift(hardwareMap, telemetry, time = new ElapsedTime(), true);
        //robot = new Robot(hardwareMap);
        drive = new MecDrive(hardwareMap, false, telemetry);
        //systems = new EndgameSystems(hardwareMap);


        //score.setLinkagePositionLogistic(Constants.linkageDown, 500);
        score.setGrabberPosition(Constants.open - 0.15);

        distance = hardwareMap.get(ColorRangeSensor.class, "distance");


        //Color sensor gain values
        //color.setGain(300);
        distance.setGain(180);




        waitForStart();



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
                drive.setPower(new Vector2D(leftStickX * Constants.SPRINT_LINEAR_MODIFIER, leftStickY * Constants.SPRINT_LINEAR_MODIFIER), gamepad1.right_stick_x * Constants.SPRINT_ROTATIONAL_MODIFIER, false);
            } else if (score.isExtended()) {
                //Slow down when slides are extended
                drive.setPower(new Vector2D(leftStickX * Constants.EXTENDED_LINEAR_MODIFIER, leftStickY * Constants.EXTENDED_LINEAR_MODIFIER), gamepad1.right_stick_x * Constants.EXTENDED_ROTATIONAL_MODIFIER, false);
            } else {
                drive.setPower(new Vector2D(leftStickX * Constants.NORMAL_LINEAR_MODIFIER, leftStickY * Constants.NORMAL_LINEAR_MODIFIER), gamepad1.right_stick_x * Constants.NORMAL_ROTATIONAL_MODIFIER, false);
            }


            //Lift Thread Stuff
            if (gamepad1.left_trigger > 0.1) {
                //score.setPower(0.2);
                if (score.getScoringMode() != ScoringSystemV2EpicLift.ScoringMode.ULTRA && !liftBrokenMode) {
                    /*TODO: Need to make different logic so that linkage is moved afterwards
                       (cuz now we have lift update so maybe set linkage position after encoder position)
                     */


                    score.commandAutoGoToPosition();


                    if (score.getScoringMode() == ScoringSystemV2EpicLift.ScoringMode.LOW) {


                                score.setLinkagePosition(0.71);


                    } else {


                                score.setLinkagePosition(Constants.linkageScoreV2 - 0.05);

                    }

                } else if (score.getScoringMode() == ScoringSystemV2EpicLift.ScoringMode.ULTRA) {

                            score.setLinkagePosition(0.15);


                }

            }


            //Scoring feature
            if (gamepad1.right_trigger > 0.1) {

                if (score.getScoringMode() != ScoringSystemV2EpicLift.ScoringMode.ULTRA) {

                    CommandScheduler.getInstance().schedule(
                            new InstantCommand(() -> score.setGrabberPosition(Constants.score)),
                            new InstantCommand(() -> new Timing.Timer(600, TimeUnit.MILLISECONDS)),

                            new ParallelCommandGroup(
                                    new SequentialCommandGroup(
                                            new InstantCommand(() -> score.setLinkagePosition(Constants.linkageUpV2)),
                                            new InstantCommand(() -> new Timing.Timer(70, TimeUnit.MILLISECONDS)),
                                            new InstantCommand(() -> score.setLinkageConeStack(true))
                                    ),

                                    new InstantCommand(() -> score.setGrabberPosition(Constants.open - 0.15))

                            )


                    );


                    //Do nothing during movement phase
                    //Reset to zero and no passive power
                    score.setLiftTarget(0);

                    //Open Grabber and reset linkage

                    //score.setLinkagePositionLogistic(Constants.linkageDownV2, 300);
                    //score.setLinkagePositionLogistic(0.8, 500);
                } else {

                    CommandScheduler.getInstance().schedule(
                            new InstantCommand(() -> score.setGrabberPosition(Constants.open - 0.15)),
                            new InstantCommand(() -> new Timing.Timer(700, TimeUnit.MILLISECONDS))
                    );

                }

                if (liftBrokenMode) {

                    CommandScheduler.getInstance().schedule(
                            new InstantCommand(() -> new Timing.Timer(2000, TimeUnit.MILLISECONDS))
                    );

                }

                //TODO: fix this
                score.lowerConeStack();

                if (scoringPattern = true) {
                    if (score.getScoringMode() == ScoringSystemV2EpicLift.ScoringMode.LOW) {
                        score.setScoringMode(ScoringSystemV2EpicLift.ScoringMode.MEDIUM);
                    } else if (score.getScoringMode() == ScoringSystemV2EpicLift.ScoringMode.MEDIUM) {
                        score.setScoringMode(ScoringSystemV2EpicLift.ScoringMode.HIGH);
                    } else if (score.getScoringMode() == ScoringSystemV2EpicLift.ScoringMode.HIGH) {
                        scoringPattern = false;
                    }
                }


                //Resetting flags
                autoLinkageFlag = true;
                grabFlag = true;


                CommandScheduler.getInstance().schedule(
                        new InstantCommand(() -> new Timing.Timer(500, TimeUnit.MILLISECONDS))
                );


                //Not extended anymore
                score.setExtended(false);

                //Automated Grab
            } else if ((distance.getNormalizedColors().red > 0.80 || distance.getNormalizedColors().blue > 0.80) && autoLinkageFlag) {




                grabFlag = false;

                CommandScheduler.getInstance().schedule(
                        new InstantCommand(() -> score.setGrabberPosition(Constants.grabbing)),
                        new InstantCommand(() -> new Timing.Timer(150, TimeUnit.MILLISECONDS))
                );

                if (score.getScoringMode() == ScoringSystemV2EpicLift.ScoringMode.ULTRA) {

                    CommandScheduler.getInstance().schedule(
                            new InstantCommand(() -> new Timing.Timer(400, TimeUnit.MILLISECONDS))
                    );

                }

                CommandScheduler.getInstance().schedule(
                        new InstantCommand(() -> new Timing.Timer(100, TimeUnit.MILLISECONDS))

                );

                if(score.getScoringMode() == ScoringSystemV2EpicLift.ScoringMode.ULTRA){
                    CommandScheduler.getInstance().schedule(
                            new InstantCommand(() -> new Timing.Timer(400, TimeUnit.MILLISECONDS)),
                            new InstantCommand(() -> score.setLinkagePosition(0.25))
                    );
                }else if(liftBrokenMode){
                    CommandScheduler.getInstance().schedule(
                            new InstantCommand(() -> new Timing.Timer(400, TimeUnit.MILLISECONDS)),
                            new InstantCommand(() -> score.setLinkagePosition(0.54))
                    );
                }else{
                    CommandScheduler.getInstance().schedule(
                            new InstantCommand(() -> new Timing.Timer(400, TimeUnit.MILLISECONDS)),
                            new InstantCommand(() -> score.setLinkagePosition(Constants.linkageScoreV2 - 0.05))
                    );
                }
                autoLinkageFlag = false;

                CommandScheduler.getInstance().run();

            }





            //TODO: see if need to fix this logic
            //Auto linkage up logic after sensing a cone


            //TODO: tune this (both raise and lower)
            //Linkage stack cone heights with dpad up and down
            if ((gamepad1.left_bumper || gamepad1.dpad_up || gamepad1.dpad_down) && changeStackFlag) {

                //Raise linkage by height of a cone (max height of 5)
                if (gamepad1.left_bumper || gamepad1.dpad_up) {
                    score.setConeStack(5);
                    score.setLinkageConeStack(false);
                    changeStackFlag = false;

                    //Lower linkage by height of a cone (min height of 1)
                } else if (gamepad1.dpad_down) {
                    score.lowerConeStack();
                    score.setLinkageConeStack(false);
                    changeStackFlag = false;

                }




            }
            if (!gamepad1.dpad_down && !gamepad1.dpad_up && !gamepad1.left_bumper) {
                changeStackFlag = true;
            }


            //Linkage up position
            if (gamepad1.left_stick_button) {
                score.setLinkagePosition(Constants.linkageScoreV2 - 0.05);

            }


            //Changing scoring modes (toggle)
            if (gamepad1.y) {
                score.setScoringMode(ScoringSystemV2EpicLift.ScoringMode.LOW);
                scoringPattern = true;

            } else if (gamepad1.x) {
                score.setScoringMode(ScoringSystemV2EpicLift.ScoringMode.MEDIUM);

            } else if (gamepad1.b) {
                score.setScoringMode(ScoringSystemV2EpicLift.ScoringMode.HIGH);

            } else if (gamepad1.a) {
                //Ultra
                score.setScoringMode(ScoringSystemV2EpicLift.ScoringMode.ULTRA);
            }


            //Manual slides (dpad right and left)
            if (gamepad1.dpad_right) {
                score.setPower(1);
            } else if (gamepad1.dpad_left) {
                score.setPower(-0.55);
            }

            //PID Update
            if (!gamepad1.dpad_right && !gamepad1.dpad_left) {
                score.newLiftPIDUpdate(0.75);
            }

            //Lift Broken Mode Toggle
            if (gamepad1.ps && optionsFlag) {
                optionsFlag = false;
                liftBrokenMode = !liftBrokenMode;
            }
            if (!gamepad1.ps) {
                optionsFlag = true;
            }

            //Manual Linkage
            if (gamepad2.dpad_up) {
                score.setLinkagePosition(score.getLeftLinkage() + 0.001);
            }

            if (gamepad2.dpad_down) {
                score.setLinkagePosition(score.getLeftLinkage() - 0.001);
            }


            CommandScheduler.getInstance().run();


        }


        //Stop
        drive.simpleBrake();

        //score.setLinkagePositionLogistic(0.25, 500);
        score.setLinkagePositionLogistic(Constants.linkageDownV2, 300, 100);
        //score.setLinkagePositionLogistic(0.8, 500);


        score.setGrabberPosition(Constants.open - 0.15);
    }
}
