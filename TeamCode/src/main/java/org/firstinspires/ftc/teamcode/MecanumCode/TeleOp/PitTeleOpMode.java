package org.firstinspires.ftc.teamcode.MecanumCode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumCode.Common.CapstoneArm;
import org.firstinspires.ftc.teamcode.MecanumCode.Common.Carousel;
import org.firstinspires.ftc.teamcode.MecanumCode.Common.Constants;
import org.firstinspires.ftc.teamcode.MecanumCode.Common.MagneticArm;
import org.firstinspires.ftc.teamcode.MecanumCode.Common.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.MecanumCode.Common.Vector2D;

import java.io.FileNotFoundException;

@TeleOp(name="Pit TeleOp Mode")
public class PitTeleOpMode extends LinearOpMode {

    MecanumDriveTrain drive;
    CapstoneArm capArm;

    Thread driveThread;
    Thread armThread;
    Thread capArmThread;

    Boolean driveSwapped = false;
    Boolean previousBackState = false;


    /**
     * calibrate all these values kevin
     */

    private final double NORMAL_LINEAR_MODIFIER = 0.45;
    private final double NORMAL_ROTATIONAL_MODIFIER = 0.45;
    private final double SPRINT_LINEAR_MODIFIER = 1;
    private final double SPRINT_ROTATIONAL_MODIFIER = 0.75;


    @Override
    public void runOpMode() throws InterruptedException {

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
        capArmThread.start();
        capArm.goToPosition(100);

        while(opModeIsActive());
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
                drive.setPower(new Vector2D(-gamepad1.left_stick_x * SPRINT_LINEAR_MODIFIER, -gamepad1.left_stick_y * SPRINT_LINEAR_MODIFIER), gamepad1.right_stick_x * SPRINT_ROTATIONAL_MODIFIER, false);
            } else {
                drive.setPower(new Vector2D(-gamepad1.left_stick_x * NORMAL_LINEAR_MODIFIER, -gamepad1.left_stick_y * NORMAL_LINEAR_MODIFIER), gamepad1.right_stick_x * NORMAL_ROTATIONAL_MODIFIER, false);
            }
        } else {
            if (gamepad1.right_bumper) { // replace this with a button for sprint
                drive.setPower(new Vector2D(gamepad1.left_stick_x * SPRINT_LINEAR_MODIFIER, gamepad1.left_stick_y * SPRINT_LINEAR_MODIFIER), gamepad1.right_stick_x * SPRINT_ROTATIONAL_MODIFIER, false);
            } else {
                drive.setPower(new Vector2D(gamepad1.left_stick_x * NORMAL_LINEAR_MODIFIER, gamepad1.left_stick_y * NORMAL_LINEAR_MODIFIER), gamepad1.right_stick_x * NORMAL_ROTATIONAL_MODIFIER, false);
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

        if(gamepad1.y) {
            capArm.goToPosition(Constants.TOP_GOAL_POS);
            sleep(1000);
            capArm.toggleGrab();
            sleep(1000);
            capArm.goToPosition(0);

        } else if(gamepad1.b) {
            capArm.goToPosition(Constants.MID_GOAL_POS);
            sleep(1000);
            capArm.toggleGrab();
            sleep(1000);
            capArm.goToPosition(0);

        } else if(gamepad1.a) {
            capArm.goToPosition(Constants.BOTTOM_GOAL_POS);
            sleep(1000);
            capArm.toggleGrab();
            sleep(1000);
            capArm.goToPosition(0);
        }





        //capArm.setPower(gamepad2.left_stick_y / 2);
    }
}
