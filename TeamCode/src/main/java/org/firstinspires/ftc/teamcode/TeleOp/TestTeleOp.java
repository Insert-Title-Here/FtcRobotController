package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Common.MecanumDrive;
import org.firstinspires.ftc.teamcode.Common.Vector2D;

@TeleOp
public class TestTeleOp extends LinearOpMode {

    MecanumDrive drive;

    //TODO: Move scoring system stuff to its own class
    DcMotor lift;
    Servo clawl;
    Servo clawr;


    private final double NORMAL_LINEAR_MODIFIER = 0.6;
    private final double NORMAL_ROTATIONAL_MODIFIER = 0.6;
    private final double SPRINT_LINEAR_MODIFIER = 1;
    private final double SPRINT_ROTATIONAL_MODIFIER = 1;


    boolean liftIsUp = false;
    int origLiftPos;
    int liftPos;
    boolean clawIsClosed = true;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new MecanumDrive(hardwareMap, telemetry);
        lift = hardwareMap.get(DcMotor.class, "lift");
        clawl = hardwareMap.get(Servo.class, "clawl");
        clawr = hardwareMap.get(Servo.class, "clawr");

        int origliftPos = lift.getCurrentPosition();
        int liftPos = lift.getCurrentPosition();


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        clawl.setPosition(0);
        clawr.setPosition(0.6);

        waitForStart();


        while(opModeIsActive()){

            if (gamepad1.right_bumper) {
                liftIsUp = false;
                drive.setPower(new Vector2D(gamepad1.right_stick_x * SPRINT_LINEAR_MODIFIER, gamepad1.left_stick_y * SPRINT_LINEAR_MODIFIER), gamepad1.left_stick_x * SPRINT_ROTATIONAL_MODIFIER, false);

            }
            else {
                drive.setPower(new Vector2D(gamepad1.right_stick_x * NORMAL_LINEAR_MODIFIER, gamepad1.left_stick_y * NORMAL_LINEAR_MODIFIER), gamepad1.left_stick_x * NORMAL_ROTATIONAL_MODIFIER, false);
            }

            if (gamepad1.left_trigger > 0.1) {
                lift.setPower(gamepad1.left_trigger*0.8);
                liftPos = lift.getCurrentPosition();
                liftIsUp = true;
            } else if (gamepad1.right_trigger > 0.1) {
                lift.setPower(-gamepad1.right_trigger*0.5);
                liftPos = lift.getCurrentPosition();
                liftIsUp = true;
            } else {
                if(liftIsUp){
                    while(opModeIsActive()&&lift.getCurrentPosition()<(liftPos-50)) {
                        lift.setPower(0.1);
                        telemetry.addData("Looping", "Adjusting lift position");
                    }
                }
                lift.setPower(0);
            }

            if(lift.getCurrentPosition()==origLiftPos){
                liftIsUp=false;
            }

            if(gamepad1.left_bumper){
                if(clawIsClosed){
                    clawr.setPosition(0);
                    clawl.setPosition(0.6);
                } else{
                    clawl.setPosition(0);
                    clawr.setPosition(0.6);
                }

                clawIsClosed = !clawIsClosed;
            }




            telemetry.addData("flPos", drive.getFLPosition());
            telemetry.addData("frPos", drive.getFRPosition());
            telemetry.addData("blPos", drive.getBLPosition());
            telemetry.addData("brPos", drive.getBRPosition());
            telemetry.addData("liftPos", lift.getCurrentPosition());
            telemetry.update();


        }

        drive.setPower(0, 0, 0, 0);
    }
}