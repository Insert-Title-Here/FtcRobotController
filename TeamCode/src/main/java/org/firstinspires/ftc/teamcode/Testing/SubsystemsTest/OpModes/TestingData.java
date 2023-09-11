package org.firstinspires.ftc.teamcode.Testing.SubsystemsTest.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Competition.Interleagues.Common.Constants;
import org.firstinspires.ftc.teamcode.Competition.Interleagues.Common.Vector2D;
import org.firstinspires.ftc.teamcode.Testing.SubsystemsTest.Subsystems.Robot;
import org.firstinspires.ftc.teamcode.Testing.SubsystemsTest.Subsystems.ScoringSystemBulk;

public class TestingData extends LinearOpMode {

    Robot robot;
    Thread liftThread, linkageThread;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        robot.score.setLiftBrokenMode(false);


        liftThread = new Thread(){
            @Override
            public void run() {
                while(opModeIsActive()){

                    //Lift up to scoring position
                    if(gamepad1.left_trigger > 0.1){
                        //robot.score.setPower(0.2);
                        if(robot.score.getScoringMode() != ScoringSystemBulk.ScoringMode.ULTRA && !robot.score.getLiftBrokenMode()) {

                            robot.score.commandAutoGoToPosition();

                            if(robot.score.getScoringMode() == ScoringSystemBulk.ScoringMode.LOW) {
                                robot.score.setLinkagePosition(0.7);
                            } else {
                                robot.score.setLinkagePosition(Constants.linkageScoreV2 - 0.05);
                            }
                            //passive = PassivePower.EXTENDED;
                        }else if (robot.score.getScoringMode() == ScoringSystemBulk.ScoringMode.ULTRA){
                            robot.score.setLinkagePosition(0.15);
                        }

                    }


                    //Scoring feature
                    if(gamepad1.right_trigger > 0.1){

                        if(robot.score.getScoringMode() != ScoringSystemBulk.ScoringMode.ULTRA) {
                            robot.score.setGrabberPosition(Constants.score);

                            try {
                                sleep(600);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }


                            robot.score.setToLinkageDown(true);
                            robot.score.setGrabberPosition(Constants.open - 0.15);


                            //Do nothing during movement phase
                            //Reset to zero and no passive power
                            robot.score.setLiftTarget(0);

                        }else{

                            robot.score.setGrabberPosition(Constants.open - 0.15);
                            try {
                                sleep(700);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }

                        }

                        if (robot.score.getLiftBrokenMode()) {
                            try {
                                sleep(2000);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                        }

                        //TODO: fix this
                        robot.score.lowerConeStack();



                        //Resetting flags
                        robot.score.setAutoLinkageFlag(true);


                        try {
                            Thread.currentThread().sleep(500);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }

                        //Not extended anymore
                        robot.score.setExtended(false);

                        //Automated Grab
                    }else if((robot.getColor().red > 0.80 || robot.getColor().red > 0.80) && robot.score.getAutoLinkageFlag()){


                        robot.score.setGrabberPosition(Constants.grabbing);



                        try {
                            Thread.currentThread().sleep(150);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }

                        if(robot.score.getScoringMode() == ScoringSystemBulk.ScoringMode.ULTRA){
                            try {
                                sleep(400);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }

                        }
                        robot.score.setToLinkageUp(true);
                        robot.score.setAutoLinkageFlag(false);


                    }



                    //TODO: see if need to fix this logic
                    //Auto linkage up logic after sensing a cone


                    //TODO: tune this (both raise and lower)
                    //Linkage stack cone heights with dpad up and down
                    if((gamepad1.dpad_up || gamepad1.dpad_down) && robot.score.getChangeStackFlag()){

                        //Raise linkage by height of a cone (max height of 5)
                        if(gamepad1.dpad_up) {
                            robot.score.setConeStack(5);
                            robot.score.setLinkageConeStack(false);
                            robot.score.setChangeStackFlag(false);

                            //Lower linkage by height of a cone (min height of 1)
                        }else if(gamepad1.dpad_down){
                            robot.score.lowerConeStack();
                            robot.score.setLinkageConeStack(false);
                            robot.score.setChangeStackFlag(false);

                        }

                        try {
                            sleep(200);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }

                    }
                    if(!gamepad1.dpad_down && !gamepad1.dpad_up){
                        robot.score.setChangeStackFlag(true);
                    }


                    //Linkage up position
                    if(gamepad1.left_stick_button){
                        robot.score.setLinkagePosition(Constants.linkageScoreV2 - 0.05);

                    }



                    //Manual open and close grabber
                    if(gamepad1.right_stick_button && robot.score.getManualFlag()){
                        if(robot.score.getGrabberPosition() != Constants.open - 0.15) {
                            robot.score.setGrabberPosition(Constants.open - 0.15);
                            try {
                                sleep(300);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }

                        }else{
                            robot.score.setGrabberPosition(Constants.grabbing);
                            try {
                                sleep(300);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }

                        }
                        robot.score.setManualFlag(false);
                    }

                    if(!gamepad1.right_stick_button){
                        robot.score.setManualFlag(true);
                    }



                    //Changing scoring modes (toggle)
                    if(gamepad1.y){
                        robot.score.setScoringMode(ScoringSystemBulk.ScoringMode.LOW);


                    }else if(gamepad1.x){
                        robot.score.setScoringMode(ScoringSystemBulk.ScoringMode.MEDIUM);

                    }else if(gamepad1.b){
                        robot.score.setScoringMode(ScoringSystemBulk.ScoringMode.HIGH);

                    }else if(gamepad1.a){
                        //Ultra
                        robot.score.setScoringMode(ScoringSystemBulk.ScoringMode.ULTRA);
                    }


                    //TODO: could look into changing the getRightEncoder
                    //Manual slides (dpad right and left)
                    if(gamepad1.dpad_right){
                        //passive = PassivePower.MOVEMENT;
                        robot.score.setPower(1);


                        robot.score.setLiftTarget(-1 * robot.score.getRightEncoderPos());

                    }else if(gamepad1.dpad_left){
                        //passive = PassivePower.MOVEMENT;
                        robot.score.setPower(-0.55);
                        robot.score.setLiftTarget(-1 * robot.score.getRightEncoderPos());

                    }else{
                        if(robot.score.getLiftTarget() == 0){
                            robot.score.newLiftPIDUpdate(0.55);
                            telemetry.addData("stuff", "slow");

                        }else {
                            robot.score.newLiftPIDUpdate(1);
                            telemetry.addData("stuff", "fast");

                        }

                    }

                    telemetry.addData("target", robot.score.getLiftTarget());

                    telemetry.update();


                    if (gamepad1.left_bumper && robot.getOptionsFlag()) {
                        robot.setOptionsFlag(false);
                        robot.score.setLiftBrokenMode(!robot.score.getLiftBrokenMode());

                        if(robot.score.getLiftBrokenMode()){
                            gamepad1.rumble(1500);
                        }else{
                            gamepad1.rumble(200);
                        }
                    }
                    if (!gamepad1.left_bumper) {
                        robot.setOptionsFlag(true);
                    }

                    if((gamepad2.dpad_up || gamepad2.dpad_down) && robot.score.getChangeToggle()){
                        if(gamepad2.dpad_up){
                            robot.score.setLinkagePosition(robot.score.getLeftLinkage() + 0.025);

                        }else{
                            robot.score.setLinkagePosition(robot.score.getLeftLinkage() - 0.025);


                        }

                        robot.score.setChangeToggle(false);


                    }

                    if(!gamepad2.dpad_up && !gamepad2.dpad_down){
                        robot.score.setChangeToggle(true);
                    }



                }

            }
        };

        //Logistic Linkage thread
        linkageThread = new Thread() {

            @Override
            public void run() {
                while(opModeIsActive()) {
                    if(robot.score.getToLinkageUp()) {

                        try {
                            Thread.currentThread().sleep(100);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                        if (robot.score.getScoringMode() == ScoringSystemBulk.ScoringMode.ULTRA) {
                            robot.score.setLinkagePosition(0.25);
                        } else if (robot.score.getLiftBrokenMode()) {
                            robot.score.setLinkagePosition(0.54);
                        } else {
                            robot.score.setLinkagePosition(Constants.linkageScoreV2 - 0.05);
                        }
                        robot.score.setToLinkageUp(false);
                    }else if(robot.score.getToLinkageDown()) {

                        robot.score.setLinkagePosition(Constants.linkageUpV2);
                        try {
                            Thread.currentThread().sleep(70);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }

                        robot.score.setLinkageConeStack(true);
                        robot.score.setToLinkageDown(false);
                    }
                }
            }
        };

        waitForStart();

        //Starting Threads
        robot.data.start();

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


            if(robot.score.isExtended()){
                //Slow down when slides are extended
                robot.drive.setPower(new Vector2D(leftStickX * Constants.EXTENDED_LINEAR_MODIFIER, leftStickY * Constants.EXTENDED_LINEAR_MODIFIER), gamepad1.right_stick_x * Constants.EXTENDED_ROTATIONAL_MODIFIER, false);
            } else{
                robot.drive.setPower(new Vector2D(leftStickX, leftStickY), gamepad1.right_stick_x * Constants.NORMAL_ROTATIONAL_MODIFIER, false);
            }


        }


        //Stop
        robot.drive.brake();

        robot.score.setLinkagePositionLogistic(Constants.linkageDownV2, 300, 100);

        robot.score.setGrabberPosition(Constants.open - 0.15);


    }
}
