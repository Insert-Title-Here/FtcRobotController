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


@TeleOp(name = "HHE TeleOP (Survival)")
public class Kids extends LinearOpMode {

    ScoringSystemNewest score;
    MecDriveV2 drive;
    ElapsedTime time = new ElapsedTime();

    ColorRangeSensor distance;


    private volatile boolean autoLinkageFlag, liftBrokenMode = true, optionsFlag = true;
    private Thread liftThread;

    @Override
    public void runOpMode() throws InterruptedException {


        //Initializing flags
        autoLinkageFlag = true;

        score = new ScoringSystemNewest(hardwareMap, telemetry, true, time);
        drive = new MecDriveV2(hardwareMap, false, telemetry, time);
        distance = hardwareMap.get(ColorRangeSensor.class, "distance");

        score.setGrabberPosition(0.13);

        //Color sensor gain values
        distance.setGain(230);


        //Lift Thread
        liftThread = new Thread() {
            @Override
            public void run() {
                while (opModeIsActive()) {


                    //Scoring feature
                    if (gamepad1.right_trigger > 0.1) {

                        try {
                            Thread.currentThread().sleep(100);
                        } catch (InterruptedException e) {
                            throw new RuntimeException(e);
                        }
                        score.setGrabberPosition(0.25);

                        try {
                            Thread.currentThread().sleep(250);
                        } catch (InterruptedException e) {
                            throw new RuntimeException(e);
                        }


                        score.setGrabberPosition(0.13);

                        try {
                            sleep(1000);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }


                        score.setLinkageConeStack(true);
                        score.lowerConeStack();


                        //Resetting flags
                        autoLinkageFlag = true;

                        try {
                            Thread.currentThread().sleep(500);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }

                        //Not extended anymore
                        score.setExtended(false);

                        //Automated Grab
                    } else if ((distance.getNormalizedColors().red > 0.90 || distance.getNormalizedColors().blue > 0.90) && autoLinkageFlag) {

                        score.setGrabberPosition(0.37);

                        try {
                            Thread.currentThread().sleep(250);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }

                        score.setLinkagePositionLogistic(0.54, 500, 100);
                        autoLinkageFlag = false;
                    }

                    //Manual open and close grabber
                    if (gamepad1.right_stick_button) {
                        score.setLinkagePositionLogistic(0.54, 500, 100);
                        autoLinkageFlag = false;
                    }

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
                }
            }
        };

        waitForStart();

        //Starting Threads
        liftThread.start();
        while (opModeIsActive()) {

            //N S E W Drive (no strafe)
            double leftStickX = 0;
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
                drive.setPower(new Vector2D(-leftStickX * Constants.SPRINT_LINEAR_MODIFIER, -leftStickY * Constants.SPRINT_LINEAR_MODIFIER), -gamepad1.right_stick_x * 0.7, false);
            } else {

                if (score.isExtended() && score.getScoringMode() != ScoringSystemNewest.ScoringMode.LOW) {
                    //Slow down when slides are extended
                    drive.setPower(new Vector2D(-leftStickX * 0, -leftStickY * 0.35), -gamepad1.right_stick_x * 0.3, false);
                } else {
                    drive.setPower(new Vector2D(-leftStickX * 0/* * Constants.NORMAL_LINEAR_MODIFIER*/, -leftStickY * 0.4/* * Constants.NORMAL_LINEAR_MODIFIER*/), -gamepad1.right_stick_x * 0.35, false);
                }

            }
        }

        //Stop
        drive.simpleBrake();
        score.setLinkagePositionLogistic(Constants.linkageDownV2, 500, 100);
        score.setGrabberPosition(0.13);
    }
}
