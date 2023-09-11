package org.firstinspires.ftc.teamcode.State.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.State.Common.TestingCHUBS;
import org.firstinspires.ftc.teamcode.State.Common.Vector2D;


public class DriveTest extends LinearOpMode {

    TestingCHUBS drive;

    private final double NORMAL_LINEAR_MODIFIER = 0.7;
    private final double NORMAL_ROTATIONAL_MODIFIER = 0.3;
    private final double SPRINT_LINEAR_MODIFIER = 1;
    private final double SPRINT_ROTATIONAL_MODIFIER = 0.75;



    @Override
    public void runOpMode() throws InterruptedException {

        drive = new MecanumDrive(hardwareMap, telemetry);


        waitForStart();

        while(opModeIsActive()){

            double gamepadX = gamepad1.left_stick_x;
            double gamepadY = gamepad1.left_stick_y;
            if(Math.abs(gamepadX) < Math.abs(gamepadY)){
                gamepadX = 0;
            }else if(Math.abs(gamepadX) > Math.abs(gamepadY)){
                gamepadY = 0;
            }else{
                gamepadX = 0;
                gamepadY = 0;
            }

            drive.setPower(new Vector2D(gamepadX * NORMAL_LINEAR_MODIFIER, gamepadY * NORMAL_LINEAR_MODIFIER), gamepad1.right_stick_x * NORMAL_ROTATIONAL_MODIFIER, false);

        }

        drive.setPower(0,0,0,0);
    }
}
