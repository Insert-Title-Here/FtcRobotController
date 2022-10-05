package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Common.MecanumDrive;
import org.firstinspires.ftc.teamcode.Common.ScoringSystem;
import org.firstinspires.ftc.teamcode.Common.Vector2D;

@TeleOp
public class FirstTeleOp extends LinearOpMode {

    //TODO: change names if you want to
    MecanumDrive drive;
    ScoringSystem score;


    private final double NORMAL_LINEAR_MODIFIER = 0.5;
    private final double NORMAL_ROTATIONAL_MODIFIER = 0.5;
    private final double SPRINT_LINEAR_MODIFIER = 1;
    private final double SPRINT_ROTATIONAL_MODIFIER = 1;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new MecanumDrive(hardwareMap, telemetry);
        score = new ScoringSystem(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        //score = new ScoringSystem(hardwareMap);

        //Open
        score.setClawPosition(0.9);



        waitForStart();


        while(opModeIsActive()){


            //TODO: Decide if you want sprint capability
            if (gamepad1.right_bumper) { // replace this with a button for sprint
                drive.setPower(new Vector2D(gamepad1.left_stick_x * SPRINT_LINEAR_MODIFIER, gamepad1.left_stick_y * SPRINT_LINEAR_MODIFIER), gamepad1.right_stick_x * SPRINT_ROTATIONAL_MODIFIER, false);
            }
            else {
                drive.setPower(new Vector2D(gamepad1.left_stick_x * NORMAL_LINEAR_MODIFIER, gamepad1.left_stick_y * NORMAL_LINEAR_MODIFIER), gamepad1.right_stick_x * NORMAL_ROTATIONAL_MODIFIER, false);
            }
            //disable trigger as you can use left right down up pad instead(pressing left trigger too much will mess up encoder values for lift system

            if(gamepad1.right_trigger > 0.1){
                score.setPower(gamepad1.right_trigger/2);
            }else if(gamepad1.left_trigger > 0.1){
                //TODO: Create method I can call that will calibrate based off of how many tics, the "40" position below
                if(score.getEncoderPosition() < 40){
                    score.setPower(0);
                }else{
                    score.setPower(-gamepad1.left_trigger/2);
                }
            }else{
                score.setPower(0);
            }



            if(gamepad1.b){
                //Closed
                score.setClawPosition(0.45);


            //2220

            }else if(gamepad1.x){
                //Open
                score.setClawPosition(0.9);

            }
            if(gamepad1.a){
                drive.resetEncoders();
            }
            if(gamepad1.y){
                score.resetLiftEncoder();
            }

            // reset   gamepad1.dpad_down
            // low cone, 13 in, 1209  gamenpad1.dpad_left
            // medium cone, 23 in, 1795 gamepad1.dpad_up
            // high cone, 33 in, 2390 gamepad1.dpad_right

            if(gamepad1.dpad_down) {
                //reset
                // 50 not zero b/c 435 motor does not have enough torque to stop the gravitational drop to zero tics
                score.goToPosition(50, 1);
            }

            if (gamepad1.dpad_left) {
                //low cone
                score.goToPosition(1209, 1);
            }

            if (gamepad1.dpad_up) {
                //medium cone
                score.goToPosition(1795, 1);
            }

            if (gamepad1.dpad_right) {
                //high cone
                score.goToPosition(2390, 1);
            }

            telemetry.addData("flPos", drive.getFLPosition());
            telemetry.addData("frPos", drive.getFRPosition());
            telemetry.addData("blPos", drive.getBLPosition());
            telemetry.addData("brPos", drive.getBRPosition());
            telemetry.addData("liftPos", score.getEncoderPosition());
            telemetry.addData("servoPosition", drive.getPosition()); // expected pos.
            telemetry.update();


        }

        drive.setPower(0, 0, 0, 0);
        score.setPower(0);
        score.setClawPosition(0.9);
    }
}
