package org.firstinspires.ftc.teamcode.League1.Testing.TeleOpTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.League1.Common.Constants;
import org.firstinspires.ftc.teamcode.League1.Common.OpModeWrapper;
import org.firstinspires.ftc.teamcode.League1.Subsystems.ScoringSystem2;

import java.io.FileNotFoundException;


@Disabled
@TeleOp
public class ArmTester extends OpModeWrapper {
    //Servo lServo, rServo, grabber;
    DcMotor rMotor, lMotor;

    //ScoringSystem2 score;
    //Constants constants;

    @Override
    protected void onInitialize() throws FileNotFoundException {
        //constants = new Constants();
        //score = new ScoringSystem2(hardwareMap, constants);

        //score.setLinkagePosition(0.03);
        //score.setGrabberPosition(0.75);

    }

    @Override
    protected void onStart() {



        while(opModeIsActive()) {


            if(gamepad1.right_trigger > 0.1){
                rMotor.setPower(gamepad1.right_trigger);
                lMotor.setPower(-gamepad1.right_trigger);


            }else if(gamepad1.left_trigger > 0.1){
                rMotor.setPower(-gamepad1.right_trigger);
                lMotor.setPower(gamepad1.right_trigger);


            }else{
                rMotor.setPower(0);
                lMotor.setPower(0);


            }


            telemetry.addData("lMotor", -1 * lMotor.getCurrentPosition());
            telemetry.addData("rMotor", rMotor.getCurrentPosition());
            telemetry.addData("rTrigger", gamepad1.right_trigger);
            telemetry.addData("lTrigger", gamepad1.left_trigger);
            telemetry.update();

        }




        
    }

    @Override
    protected void onStop() {



        rMotor.setPower(0);
        lMotor.setPower(0);

    }
}
