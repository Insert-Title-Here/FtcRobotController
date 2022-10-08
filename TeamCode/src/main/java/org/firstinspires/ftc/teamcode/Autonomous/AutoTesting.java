package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Common.MecanumDrive;
import org.firstinspires.ftc.teamcode.Common.Vector2D;

@Autonomous
public class AutoTesting extends LinearOpMode {

    MecanumDrive drive;

    //TODO: Move scoring system stuff to its own class
    DcMotor fr, fl, br, bl, lift;
    Servo clawl;
    Servo clawr;


    private final double NORMAL_LINEAR_MODIFIER = 0.6;
    private final double NORMAL_ROTATIONAL_MODIFIER = 0.6;
    private final double SPRINT_LINEAR_MODIFIER = 1;
    private final double SPRINT_ROTATIONAL_MODIFIER = 1;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new MecanumDrive(hardwareMap, telemetry);
        fr = hardwareMap.get(DcMotor.class, "fr");
        fl = hardwareMap.get(DcMotor.class, "fl");
        br = hardwareMap.get(DcMotor.class, "br");
        bl = hardwareMap.get(DcMotor.class, "bl");

        lift = hardwareMap.get(DcMotor.class, "lift");
        clawl = hardwareMap.get(Servo.class, "clawl");
        clawr = hardwareMap.get(Servo.class, "clawr");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        clawl.setPosition(0);
        clawr.setPosition(0.6);

        waitForStart();
        drive.setPower(new Vector2D(gamepad1.right_stick_x * NORMAL_LINEAR_MODIFIER, gamepad1.left_stick_y * NORMAL_LINEAR_MODIFIER), gamepad1.left_stick_x * NORMAL_ROTATIONAL_MODIFIER, false);
        //lift.setPower();
        //lift.setPower();
        lift.setPower(0);


        for(int i = 0; i < 20; i++) {
            clawl.setPosition(0);
            clawr.setPosition(0.6);
            sleep(200);
            clawr.setPosition(0);
            clawl.setPosition(0.6);
            sleep(200);

        }



        telemetry.addData("flPos", drive.getFLPosition());
        telemetry.addData("frPos", drive.getFRPosition());
        telemetry.addData("blPos", drive.getBLPosition());
        telemetry.addData("brPos", drive.getBRPosition());
        telemetry.update();



        drive.setPower(0,0,0,0);
    }
}
