package org.firstinspires.ftc.teamcode.League1.Testing.TeleOpTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.League1.Common.Constants;
import org.firstinspires.ftc.teamcode.League1.Common.OpModeWrapper;
import org.firstinspires.ftc.teamcode.League1.Subsystems.ScoringSystem;
import org.firstinspires.ftc.teamcode.League1.Subsystems.ScoringSystem2;

import java.io.FileNotFoundException;


@Disabled
@TeleOp
public class ArmTester extends OpModeWrapper {
    //Servo lServo, rServo, grabber;
    //DcMotor rMotor, lMotor;

    ScoringSystem2 score;
    Constants constants;

    @Override
    protected void onInitialize() throws FileNotFoundException {
        constants = new Constants();
        score = new ScoringSystem2(hardwareMap, constants);

        score.setLinkagePosition(0.03);
        score.setGrabberPosition(0.75);

    }

    @Override
    protected void onStart() {



        while(opModeIsActive()) {


            if(gamepad1.right_trigger > 0.1){
                score.setPower(gamepad1.right_trigger);

            }else if(gamepad1.left_trigger > 0.1){
                score.setPower(-gamepad1.left_trigger);

            }else{
                score.setPower(0);

            }


            telemetry.addData("Linkage Pos:", score.getRightLinkage());
            telemetry.addData("Grabber Pos:", score.getGrabberPosition());
            telemetry.addData("lMotor", -1 * score.getLeftEncoderPos());
            telemetry.addData("rMotor", score.getRightEncoderPos());
            telemetry.addData("trigger", gamepad1.right_trigger);
            telemetry.update();

        }




        
    }

    @Override
    protected void onStop() {
        /*rServo.setPosition(0);
        lServo.setPosition(0);

         */

        score.setLinkagePosition(0.1);
        sleep(200);
        score.setLinkagePosition(0.03);
        score.setGrabberPosition(0.75);
        score.setPower(0);


    }
}
