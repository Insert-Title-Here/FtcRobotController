package org.firstinspires.ftc.teamcode.MecanumCode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumCode.Common.Carousel;
import org.firstinspires.ftc.teamcode.MecanumCode.Common.MagneticArm;
import org.firstinspires.ftc.teamcode.MecanumCode.Common.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.MecanumCode.Common.Vector2D;

import java.io.FileNotFoundException;

@TeleOp(name="MecanumOpMode")
public class OmniDirectionalTeleOp extends LinearOpMode {

    MagneticArm arm;

    MecanumDriveTrain drive;

    Carousel carousel;

    Thread driveThread;
    Thread armThread;


    /**
     * calibrate all these values kevin
     */

    private final double NORMAL_LINEAR_MODIFIER = 0.5;
    private final double NORMAL_ROTATIONAL_MODIFIER = 0.5;
    private final double SPRINT_LINEAR_MODIFIER = 1;
    private final double SPRINT_ROTATIONAL_MODIFIER = 1;


    @Override
    public void runOpMode() throws InterruptedException {
        arm = new MagneticArm(hardwareMap);

        carousel = new Carousel(hardwareMap);

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

        waitForStart();
        startOpMode();
    }

    private void startOpMode() {
        driveThread.start();
        armThread.start();
        arm.setArmPosition(-100);
        while(opModeIsActive());
    }

    private void armUpdate() {
        /**
         * Kevin go implement this
         */

        if(gamepad1.left_bumper) {
            carousel.spinCarousel(4000, this);
        }

        if(gamepad1.dpad_up) {
            arm.increaseLevelPosition(0.01);
            sleep(10);
        }

        if(gamepad1.dpad_down) {
            arm.decreaseLevelPosition(0.01);
            sleep(10);
        }

        if(gamepad1.a) {
            // Fully extend arm
            //arm.setArmPositionSM(350, OmniDirectionalTeleOp.this);
            arm.setArmPosition(-350);
            while(gamepad1.a) {

            }
        }

        if(gamepad1.b) {
            // Lower level to cube height
            //arm.setLevelPosition(0.55);
            arm.setLevelPosition(arm.getLevelPosition());
        }

        if(gamepad1.y) {
            // Raise level
            arm.setLevelPosition(1);
        }

        if(gamepad1.x) {
            // Drop cube and retract arm
            arm.setMagnetPosition(MagneticArm.magnetState.OPEN);
            sleep(1000);
            arm.setMagnetPosition(MagneticArm.magnetState.GRABBING);
            //arm.setArmPositionSM(0, OmniDirectionalTeleOp.this);
            arm.setArmPosition(-100);
        }

        if (gamepad1.left_trigger > 0.1) {
            //arm.setExtensionSMPower(gamepad1.left_trigger);
            arm.setExtensionPower(gamepad1.left_trigger);
        } else if (gamepad1.right_trigger > 0.1) {
            //arm.setExtensionSMPower(-gamepad1.right_trigger);
            arm.setExtensionPower(-gamepad1.right_trigger);
        } else {
            //arm.setExtensionSMPower(0);
            arm.setExtensionPower(0);
        }
        telemetry.addData("Arm Tics", arm.getEncoderTics());
        telemetry.addData("Level Position: ", arm.getTelemetry()[0]);
        telemetry.addData("Level Position Actual", arm.getTelemetry()[1]);
        telemetry.addData("Magnet Position", arm.getTelemetry()[2]);

        telemetry.addData("FL Tics", drive.fl.getCurrentPosition());
        telemetry.addData("FR Tics", drive.fr.getCurrentPosition());
        telemetry.addData("BL Tics", drive.bl.getCurrentPosition());
        telemetry.addData("BR Tics", drive.br.getCurrentPosition());

        telemetry.update();
    }

    private void driveUpdate() {

        if(gamepad1.right_bumper) { // replace this with a button for sprint
            drive.setPower(new Vector2D(gamepad1.left_stick_x * SPRINT_LINEAR_MODIFIER, gamepad1.left_stick_y * SPRINT_LINEAR_MODIFIER), gamepad1.right_stick_x * SPRINT_ROTATIONAL_MODIFIER);
        }else{
            drive.setPower(new Vector2D(gamepad1.left_stick_x * NORMAL_LINEAR_MODIFIER, gamepad1.left_stick_y * NORMAL_LINEAR_MODIFIER), gamepad1.right_stick_x * NORMAL_ROTATIONAL_MODIFIER);
        }

        if(gamepad1.left_stick_button){
            drive.driveAuto(0.7, -3665, MecanumDriveTrain.MovementType.ROTATE);
            while(gamepad1.left_stick_button) {

            }
        }


    }
}
