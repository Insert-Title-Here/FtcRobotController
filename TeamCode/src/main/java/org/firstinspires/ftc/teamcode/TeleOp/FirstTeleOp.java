package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Common.MecanumDrive;
import org.firstinspires.ftc.teamcode.Common.Vector2D;

@TeleOp
public class FirstTeleOp extends LinearOpMode {

    //TODO: change names if you want to
    MecanumDrive drive;
    //ScoringSystem score;
    CRServo liftMotor;
    Servo claw;

    private final double NORMAL_LINEAR_MODIFIER = 0.45;
    private final double NORMAL_ROTATIONAL_MODIFIER = 0.45;
    private final double SPRINT_LINEAR_MODIFIER = 1;
    private final double SPRINT_ROTATIONAL_MODIFIER = 1;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new MecanumDrive(hardwareMap, telemetry);
        liftMotor = hardwareMap.get(CRServo.class, "liftMotor");
        claw = hardwareMap.get(Servo.class, "claw");


        //score = new ScoringSystem(hardwareMap);
        claw.setPosition(0.23);



        waitForStart();


        while(opModeIsActive()){

            //TODO: Decide if you want sprint capability
            if (gamepad1.right_bumper) { // replace this with a button for sprint
                drive.setPower(new Vector2D(gamepad1.left_stick_x * SPRINT_LINEAR_MODIFIER, gamepad1.left_stick_y * SPRINT_LINEAR_MODIFIER), gamepad1.right_stick_x * SPRINT_ROTATIONAL_MODIFIER, false);
            }

            else {
                drive.setPower(new Vector2D(gamepad1.left_stick_x * NORMAL_LINEAR_MODIFIER, gamepad1.left_stick_y * NORMAL_LINEAR_MODIFIER), gamepad1.right_stick_x * NORMAL_ROTATIONAL_MODIFIER, false);
            }

            if(gamepad1.right_trigger > 0.1){
                liftMotor.setPower(gamepad1.right_trigger / 2.3);
            }else if(gamepad1.left_trigger > 0.1){
                liftMotor.setPower(-gamepad1.left_trigger / 2.3);
            }else{
                liftMotor.setPower(0);
            }

            if(gamepad1.a){
                claw.setPosition(0.14);

            }else if(gamepad1.b){
                claw.setPosition(0.23);
            }

        }

        drive.setPower(0, 0, 0, 0);
        //score.setPower(0);

        //TODO:figure out this value
        //score.setClawPosition();
    }
}
