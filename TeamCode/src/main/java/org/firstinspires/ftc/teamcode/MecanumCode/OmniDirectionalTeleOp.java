package org.firstinspires.ftc.teamcode.MecanumCode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MagneticArm;

@TeleOp(name="MecanumOpMode")
public class OmniDirectionalTeleOp extends LinearOpMode {

    MagneticArm arm;
    MecanumDriveTrain drive;

    Thread driveThread;
    Thread armThread;


    /**
     * calibrate all these values kevin
     */

    private final double NORMAL_LINEAR_MODIFIER = 0;
    private final double NORMAL_ROTATIONAL_MODIFIER = 0;
    private final double SPRINT_LINEAR_MODIFIER = 0;
    private final double SPRINT_ROTATIONAL_MODIFIER = 0;


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
    }

    private void driveUpdate() {

        if(false) { // replace this with a button for sprint
            drive.setPower(new Vector2D(gamepad1.left_stick_x * SPRINT_LINEAR_MODIFIER, gamepad1.left_stick_y * SPRINT_LINEAR_MODIFIER), gamepad1.right_stick_x * SPRINT_ROTATIONAL_MODIFIER);
        }else{
            drive.setPower(new Vector2D(gamepad1.left_stick_x * NORMAL_LINEAR_MODIFIER, gamepad1.left_stick_y * NORMAL_LINEAR_MODIFIER), gamepad1.right_stick_x * NORMAL_ROTATIONAL_MODIFIER);
        }

    }
}
