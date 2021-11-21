package org.firstinspires.ftc.teamcode.MecanumCode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumCode.Common.MagneticArm;
import org.firstinspires.ftc.teamcode.MecanumCode.Common.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.MecanumCode.Common.Vector2D;

@TeleOp(name="MecanumOpMode")
public class OmniDirectionalTeleOp extends LinearOpMode {

    MagneticArm arm;
    MecanumDriveTrain drive;

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
        drive = new MecanumDriveTrain(hardwareMap);

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
        while(opModeIsActive());
    }

    private void armUpdate() {
        /**
         * Kevin go implement this
         */

        if(gamepad1.dpad_up) {
            arm.increaseLevelPosition(0.1);
            while(gamepad1.dpad_up) {

            }
        }

        if(gamepad1.dpad_down) {
            arm.decreaseLevelPosition(0.1);
            while(gamepad1.dpad_down) {

            }
        }

        if(gamepad1.a) {
            // Fully extend arm
            arm.setArmPosition(-300);
            while(gamepad1.a) {

            }
        }

        if(gamepad1.b) {
            // Lower level to cube height
            arm.setLevelPosition(0.5);
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
            arm.setArmPosition(0);
        }

        if (gamepad1.left_trigger > 0.1) {
            arm.setExtensionPower(gamepad1.left_trigger);
        } else if (gamepad1.right_trigger > 0.1) {
            arm.setExtensionPower(-gamepad1.right_trigger);
        } else {
            arm.setExtensionPower(0);
        }
    }

    private void driveUpdate() {

        if(gamepad1.right_bumper) { // replace this with a button for sprint
            drive.setPower(new Vector2D(gamepad1.left_stick_x * SPRINT_LINEAR_MODIFIER, gamepad1.left_stick_y * SPRINT_LINEAR_MODIFIER), gamepad1.right_stick_x * SPRINT_ROTATIONAL_MODIFIER);
        }else{
            drive.setPower(new Vector2D(gamepad1.left_stick_x * NORMAL_LINEAR_MODIFIER, gamepad1.left_stick_y * NORMAL_LINEAR_MODIFIER), gamepad1.right_stick_x * NORMAL_ROTATIONAL_MODIFIER);
        }

    }
}
