package org.firstinspires.ftc.teamcode.League1.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.League1.Common.Constants;
import org.firstinspires.ftc.teamcode.League1.Common.Robot;
import org.firstinspires.ftc.teamcode.League1.Subsystems.ScoringSystem;


//TODO: figure out bulk read

@TeleOp
public class SimpleScoringSystemTest extends LinearOpMode {

    ScoringSystem score;
    Constants constants;

    @Override
    public void runOpMode() throws InterruptedException {
        constants = new Constants();
        score = new ScoringSystem(hardwareMap,constants, false);


        waitForStart();

        while(opModeIsActive()){

            if(gamepad1.right_trigger > 0.1){
                score.setPower(gamepad1.right_trigger);

            }else if(gamepad1.left_trigger > 0.1){
                score.setPower(-gamepad1.left_trigger);
            }else{
                score.setPower(0);
            }


            if(gamepad1.dpad_down){
                //score.linkageAutomated(false);
                score.setLinkagePosition(0.05);

            }

            if(gamepad1.dpad_up){
                //score.linkageAutomated(true);
                score.setLinkagePosition(0.42);

            }

            if(gamepad1.dpad_right){
                score.setLinkagePosition(0.95);
            }

            if(gamepad1.a){
                score.grabberOpenAndClose(false);
            }

            if(gamepad1.b){
                score.grabberOpenAndClose(true);
            }

            if(gamepad1.x){
                score.reset();
            }

            if(gamepad1.y){
                score.moveToPosition(550, 0.5);
            }

            if(!score.isBusy()){
                score.setPower(0.04);

            }

            telemetry.addData("rMotor", score.getEncoderPosition(true));
            telemetry.addData("lMotor", score.getEncoderPosition(false));
            telemetry.update();

        }
    }
}
