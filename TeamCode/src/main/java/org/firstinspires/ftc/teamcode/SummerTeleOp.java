package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.io.FileNotFoundException;

@TeleOp(name="Summer Testing TeleOp")
public class SummerTeleOp extends LinearOpMode {

    Thread driveThread;
    MecanumDriveTrain drive;

    private final double NORMAL_LINEAR_MODIFIER = 0.45;
    private final double NORMAL_ROTATIONAL_MODIFIER = 0.45;
    private final double SPRINT_LINEAR_MODIFIER = 1;
    private final double SPRINT_ROTATIONAL_MODIFIER = 0.75;

    @Override
    public void runOpMode() throws InterruptedException{

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

        Servo lJoint = hardwareMap.get(Servo.class, "lJoint");
        Servo rJoint = hardwareMap.get(Servo.class, "rJoint");
        CRServo lIntake = hardwareMap.get(CRServo.class, "lIntake");
        CRServo rIntake = hardwareMap.get(CRServo.class, "rIntake");

        lJoint.setPosition(1);
        rJoint.setPosition(0);

        waitForStart();

        driveThread.start();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                lJoint.setPosition(0.9);
                rJoint.setPosition(0.1);
            } else {
                lJoint.setPosition(1);
                rJoint.setPosition(0);
            }

            if (gamepad1.right_trigger > 0.1) {
                lIntake.setPower(-gamepad1.right_trigger);
                rIntake.setPower(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0.1){
                lIntake.setPower(gamepad1.left_trigger);
                rIntake.setPower(-gamepad1.left_trigger);
            }

        }

    }

    private void driveUpdate() {
        if (gamepad1.right_bumper) { // replace this with a button for sprint
            drive.setPower(new Vector2D(gamepad1.left_stick_x * SPRINT_LINEAR_MODIFIER, gamepad1.left_stick_y * SPRINT_LINEAR_MODIFIER), gamepad1.right_stick_x * SPRINT_ROTATIONAL_MODIFIER, false);
        } else {
            drive.setPower(new Vector2D(gamepad1.left_stick_x * NORMAL_LINEAR_MODIFIER, gamepad1.left_stick_y * NORMAL_LINEAR_MODIFIER), gamepad1.right_stick_x * NORMAL_ROTATIONAL_MODIFIER, false);
        }
    }
}
