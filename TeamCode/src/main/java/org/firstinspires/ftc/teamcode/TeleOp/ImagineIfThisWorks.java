package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Common.MaintainLiftPosition;
import org.firstinspires.ftc.teamcode.Common.MecanumDrive;
import org.firstinspires.ftc.teamcode.Common.Vector2D;

@TeleOp
public class ImagineIfThisWorks extends LinearOpMode {

    MecanumDrive drive;
    Thread scoringThread;
    DcMotor turret, lift;
    Servo claw;

    private final double NORMAL_LINEAR_MODIFIER = 0.6;
    private final double NORMAL_ROTATIONAL_MODIFIER = 0.6;
    private final double SPRINT_LINEAR_MODIFIER = 1;
    private final double SPRINT_ROTATIONAL_MODIFIER = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap, telemetry);
        turret = hardwareMap.get(DcMotor.class, "Turret");
        lift = hardwareMap.get(DcMotor.class, "SlideString");
        claw = hardwareMap.get(Servo.class, "claw");

        scoringThread = new Thread(){
            @Override
            public void run() {
                while(opModeIsActive()) {
                    if (gamepad1.dpad_left) {
                        claw.setPosition(0.3);
                        try {
                            Thread.currentThread().sleep(300);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                    } else if (gamepad1.dpad_right) {
                        claw.setPosition(0);
                        try {
                            Thread.currentThread().sleep(300);
                        } catch (InterruptedException e) {
                            e.printStackTrace();
                        }
                    }

                    if (gamepad1.left_trigger > 0.1) {
                        turret.setPower(-gamepad1.left_trigger);
                    } else if (gamepad1.right_trigger > 0.1) {
                        turret.setPower(0.8 * gamepad1.right_trigger);
                    } else {
                        turret.setPower(0);
                    }
                }


            }
        };



        waitForStart();

        scoringThread.start();

        while (opModeIsActive()) {
            if (gamepad1.right_bumper) {
                drive.setPower(new Vector2D(gamepad1.left_stick_x * SPRINT_LINEAR_MODIFIER, gamepad1.left_stick_y * SPRINT_LINEAR_MODIFIER), gamepad1.right_stick_x * SPRINT_ROTATIONAL_MODIFIER, false);
            }
            else {
                drive.setPower(new Vector2D(gamepad1.left_stick_x * NORMAL_LINEAR_MODIFIER, gamepad1.left_stick_y * NORMAL_LINEAR_MODIFIER), gamepad1.right_stick_x * NORMAL_ROTATIONAL_MODIFIER, false);
            }



            /*if(gamepad1.left_bumper){
                claw.setPosition(clawIsClosed ? 0:0.3);
                sleep(300);
                clawIsClosed = !clawIsClosed;
            }
*/


            if (gamepad1.dpad_up) {
                lift.setPower(0.8);
                telemetry.addData("liftPos", lift.getCurrentPosition());
                telemetry.addLine("Lift is going up at " + gamepad1.left_trigger * 0.8 + " power");

            } else if (gamepad1.dpad_down) {
                lift.setPower(-0.6);
                telemetry.addData("liftPos", lift.getCurrentPosition());
                telemetry.addLine("Lift is going down at " + -gamepad1.right_trigger * 0.6 + " power");

            } else{
                lift.setPower(0.07);
            }

            telemetry.addData("flPos", drive.getFLPosition());
            telemetry.addData("frPos", drive.getFRPosition());
            telemetry.addData("blPos", drive.getBLPosition());
            telemetry.addData("brPos", drive.getBRPosition());
            telemetry.addData("liftPos", lift.getCurrentPosition());
            telemetry.addData("turretPos", turret.getCurrentPosition());
            telemetry.addData("Claw posititon: ", claw.getPosition());

            telemetry.update();

        }
        drive.setPower(0,0,0,0);
    }
}