package org.firstinspires.ftc.teamcode.League1.Testing.TeleOpTesting;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.League1.Common.Constants;
import org.firstinspires.ftc.teamcode.League1.Common.OpModeWrapper;
import org.firstinspires.ftc.teamcode.League1.Common.Vector2D;
import org.firstinspires.ftc.teamcode.League1.Subsystems.MecDrive;
import org.firstinspires.ftc.teamcode.League1.Subsystems.ScoringSystem2;
import org.firstinspires.ftc.teamcode.V2.NewSubsystem.ScoringSystemV2;

import java.io.FileNotFoundException;


@TeleOp
public class ArmTester extends OpModeWrapper {
    //Servo lServo, rServo, grabber;
    //DcMotor rMotor, lMotor;
    MecDrive drive;
    ScoringSystemV2 score;
    Constants constants;

    Thread armThread;


    @Override
    protected void onInitialize() throws FileNotFoundException {
        //constants = new Constants();
        score = new ScoringSystemV2(hardwareMap, constants);

        constants = new Constants();
        drive = new MecDrive(hardwareMap, false, telemetry);

        armThread = new Thread(){
            @Override
            public void run() {
                score.setLinkagePosition(0.72);

                while(opModeIsActive()){
                    if(gamepad1.right_trigger > 0.1){
                        score.moveToPosition(0, 0.5);



                    }else if(gamepad1.left_trigger > 0.1){
                        score.autoGoToPosition();



                    }

                    if(gamepad1.a){
                        score.setScoringMode(ScoringSystemV2.ScoringMode.ULTRA);
                    }else if(gamepad1.b){
                        score.setScoringMode(ScoringSystemV2.ScoringMode.HIGH);
                    }else if(gamepad1.y){
                        score.setScoringMode(ScoringSystemV2.ScoringMode.LOW);
                    }else if(gamepad1.x){
                        score.setScoringMode(ScoringSystemV2.ScoringMode.MEDIUM);
                    }


                    if(gamepad1.dpad_left){
                        score.setPower(-0.5);
                    }else if(gamepad1.dpad_right){
                        score.setPower(0.5);
                    }else{
                        score.setPower(0);
                    }
                }
            }
        };


        while(opModeInInit()){
            telemetry.addData("Linkage Pos", score.getRightLinkage());
            telemetry.update();
        }

        /*rMotor = hardwareMap.get(DcMotor.class, "RightLift");
        lMotor = hardwareMap.get(DcMotor.class, "LeftLift");

        rMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);*/



        //score.setLinkagePosition(0.03);
        //score.setGrabberPosition(0.75);

    }

    @Override
    protected void onStart() {
        armThread.start();




        while(opModeIsActive()) {

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

            if (gamepad1.right_bumper) {
                drive.setPower(new Vector2D(leftStickX * constants.SPRINT_LINEAR_MODIFIER, leftStickY * constants.SPRINT_LINEAR_MODIFIER), gamepad1.right_stick_x * constants.SPRINT_ROTATIONAL_MODIFIER, false);
            } else{
                drive.setPower(new Vector2D(leftStickX * constants.NORMAL_LINEAR_MODIFIER, leftStickY * constants.NORMAL_LINEAR_MODIFIER), gamepad1.right_stick_x * constants.NORMAL_ROTATIONAL_MODIFIER, false);
            }






            telemetry.addData("lMotor", -1 * score.getLeftEncoderPos());
            telemetry.addData("rMotor", score.getRightEncoderPos());
            telemetry.addData("fl", drive.getFLEncoder());
            telemetry.addData("fr", drive.getFREncoder());
            telemetry.addData("bl", drive.getBLEncoder());
            telemetry.addData("br", drive.getBREncoder());
            telemetry.addData("rTrigger", gamepad1.right_trigger);
            telemetry.addData("lTrigger", gamepad1.left_trigger);
            telemetry.addData("Linkage Pos", score.getLeftLinkage());
            telemetry.update();

        }




        
    }

    @Override
    protected void onStop() {
        score.setPower(0);
    }
}
