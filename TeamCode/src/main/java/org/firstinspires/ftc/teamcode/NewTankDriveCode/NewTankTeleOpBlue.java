package org.firstinspires.ftc.teamcode.NewTankDriveCode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumCode.Common.CapstoneArm;
import org.firstinspires.ftc.teamcode.MecanumCode.Common.Carousel;
import org.firstinspires.ftc.teamcode.MecanumCode.Common.Constants;
import org.firstinspires.ftc.teamcode.MecanumCode.Common.MagneticArm;
import org.firstinspires.ftc.teamcode.MecanumCode.Common.MecanumDriveTrain;

import java.io.FileNotFoundException;




@TeleOp(name="Tank Drive Blue")
public class NewTankTeleOpBlue extends LinearOpMode {
    //Go from 0 to 0.2

    MagneticArm arm;  
    MecanumDriveTrain drive;
    Carousel carousel;
    CapstoneArm capArm;

    Thread driveThread;
    Thread armThread;
    Thread capArmThread;

    Boolean driveSwapped = false;
    Boolean previousBackState = false;
    Boolean previousStartState = false;


    /**
     * calibrate all these values kevin
     */

    private final double NORMAL_LINEAR_MODIFIER = 0.45;
    private final double NORMAL_ROTATIONAL_MODIFIER = 0.3;
    private final double SPRINT_LINEAR_MODIFIER = 1;
    private final double SPRINT_ROTATIONAL_MODIFIER = 0.75;


    @Override
    public void runOpMode() throws InterruptedException {
        arm = new MagneticArm(hardwareMap, MagneticArm.OpMode.TeleOp);
        carousel = new Carousel(hardwareMap);
        capArm = new CapstoneArm(hardwareMap);

        try {
            drive = new MecanumDriveTrain(hardwareMap);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }

        driveThread = new Thread(){
            @Override
            public void run(){
                while(opModeIsActive()){
                    driveUpdate();
                }
            }
        };


        armThread = new Thread(){
            @Override
            public void run(){
                while(opModeIsActive()){
                    armUpdate();
                }
            }
        };

        capArmThread = new Thread(){
            @Override
            public void run(){
                while(opModeIsActive()){
                    capArmUpdate();
                }
            }
        };

        waitForStart();
        startOpMode();
    }

    private void startOpMode() {
        driveThread.start();
        armThread.start();
        capArmThread.start();
        capArm.goToPosition(100);

        //arm.setArmPosition(0.5);
        while(opModeIsActive());
    }

    private void armUpdate() {
        /**
         * Kevin go implement this
         */

        if(gamepad1.left_bumper) {
            carousel.spinCarousel(-4000, this, Carousel.CarouselMode.TELEOP);
        }



        if(gamepad1.a) {
            // Fully extend arm
            //arm.setArmPositionSM(350, OmniDirectionalTeleOp.this);
            arm.setArmPosition(0.985);

        }

        if(gamepad1.b) {
            // Lower level to cube height

            arm.setArmPosition(Constants.MAGARM_FREIGHT);
            arm.setLevelPosition(Constants.LEVEL_DOWN_POS);
            //arm.setLevelPosition(arm.getLevelPosition());
        }

        if(gamepad1.y) {
            // Raise level


            if(arm.levelPosition == Constants.LEVEL_HALF_POS) {
                arm.setArmPosition(Constants.NEW_MAGARM_EXTENDED);
                arm.setLevelPosition(Constants.LEVEL_UP_POS);
            }else if(arm.levelPosition != Constants.LEVEL_HALF_POS && arm.levelPosition != Constants.LEVEL_UP_POS){
                arm.setLevelPosition(Constants.LEVEL_HALF_POS);
            }

        }

        if(gamepad1.x) {
            // Drop cube and retract arm
            arm.setMagnetPosition(MagneticArm.magnetState.OPEN);
            sleep(1000);
            arm.setMagnetPosition(MagneticArm.magnetState.GRABBING);
            //arm.setArmPositionSM(0, OmniDirectionalTeleOp.this);
            arm.setArmPosition(Constants.MAGARM_FREIGHT);
        }

        if (gamepad1.left_trigger > 0.1) {
            //arm.setExtensionSMPower(gamepad1.left_trigger);
            arm.manualExtension(false);
        } else if (gamepad1.right_trigger > 0.1) {
            //arm.setExtensionSMPower(-gamepad1.right_trigger);
            arm.manualExtension(true);
        }





        if(gamepad1.start && !previousStartState) {
            capArm.toggleGrab();
            previousStartState = true;
        }
        if(!gamepad1.start) {
            previousStartState = false;
        }


        telemetry.addData("Arm Tics", arm.getArmPosition());
        telemetry.addData("Level Position: ", arm.getTelemetry()[0]);
        telemetry.addData("Level Position Actual", arm.getTelemetry()[1]);
        telemetry.addData("Magnet Position", arm.getTelemetry()[2]);

        telemetry.addData("Capstone Arm Tics: ", capArm.getTelemetry()[0]);
        telemetry.addData("Capstone Servo Position: ", capArm.getTelemetry()[1]);


        telemetry.addData("FL Tics", drive.fl.getCurrentPosition());
        telemetry.addData("FR Tics", drive.fr.getCurrentPosition());
        telemetry.addData("BL Tics", drive.bl.getCurrentPosition());
        telemetry.addData("BR Tics", drive.br.getCurrentPosition());

        telemetry.update();
    }

    private void driveUpdate() {

        if(gamepad1.back && !previousBackState) {
            driveSwapped = !driveSwapped;
            previousBackState = true;
        }
        if(!gamepad1.back) {
            previousBackState = false;
        }


        if(driveSwapped) {
            if (gamepad1.right_bumper) { // replace this with a button for sprint
                drive.tankSetPower(gamepad1.left_stick_y * SPRINT_LINEAR_MODIFIER, gamepad1.right_stick_x * SPRINT_ROTATIONAL_MODIFIER);
            } else {
                drive.tankSetPower(gamepad1.left_stick_y * NORMAL_LINEAR_MODIFIER, gamepad1.right_stick_x * NORMAL_ROTATIONAL_MODIFIER);
            }
        } else {
            if (gamepad1.right_bumper) { // replace this with a button for sprint
                drive.tankSetPower(-gamepad1.left_stick_y * SPRINT_LINEAR_MODIFIER, gamepad1.right_stick_x * SPRINT_ROTATIONAL_MODIFIER);
            } else {
                drive.tankSetPower(-gamepad1.left_stick_y * NORMAL_LINEAR_MODIFIER, gamepad1.right_stick_x * NORMAL_ROTATIONAL_MODIFIER);
            }
        }



        /*if (gamepad1.left_stick_button) {
            drive.driveAuto(0.7, -3665, MecanumDriveTrain.MovementType.ROTATE);
            while (gamepad1.left_stick_button) {

            }
        }

         */
    }

    private void capArmUpdate() {

        if(gamepad1.dpad_up) {
            capArm.goToPosition(Constants.CAPPING_POS);
        } else if(gamepad1.dpad_down) {
            capArm.goToPosition(0);
        }

        if(gamepad1.dpad_right && capArm.getTelemetry()[0] > Constants.MAX_MANUAL_CAP) {
            //driveSwapped = true;
            capArm.setPower(-0.3);
        }else if(gamepad1.dpad_left && capArm.getTelemetry()[0] < 0) {
            //driveSwapped = false;
            capArm.setPower(0.3);
        }else{
            capArm.setPower(0);
        }



        //capArm.setPower(gamepad2.left_stick_y / 2);
    }
}
