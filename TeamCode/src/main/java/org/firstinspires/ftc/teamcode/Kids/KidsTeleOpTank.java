package org.firstinspires.ftc.teamcode.Kids;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Common.Constants;
import org.firstinspires.ftc.teamcode.Common.DriveTrainClassBot;
import org.firstinspires.ftc.teamcode.Common.MecDriveV2;
import org.firstinspires.ftc.teamcode.Common.ScoringSystem;
import org.firstinspires.ftc.teamcode.Common.ScoringSystemClassBot;
import org.firstinspires.ftc.teamcode.Common.Vector2D;


@TeleOp
public class KidsTeleOpTank extends LinearOpMode {

    DriveTrainClassBot drive;
    ScoringSystemClassBot score;

    public void runOpMode() {

        drive = new DriveTrainClassBot(hardwareMap);
        score = new ScoringSystemClassBot(hardwareMap);

        waitForStart();

        while(opModeIsActive()) {
            double drivePow = gamepad1.right_stick_x;
            double rotatePow = gamepad1.left_stick_y;
            double leftPower = Range.clip(drivePow - rotatePow, -1, 1);
            double rightPower = Range.clip(drivePow + rotatePow, -1, 1);
            drive.setPower(0.5*leftPower, 0.5*rightPower);

            if(gamepad1.left_trigger > 0.1){
                //starting down pos
                //score.goToPosArm(0,0.3);
            }
            if(gamepad1.right_trigger > 0.1){
                //upward posss
               // score.goToPosArm(-100, 0.3);
            }
            if(gamepad1.left_bumper){
                //open
                //score.goToPosGuard(0);
            }
            if(gamepad1.right_bumper){
                drive.setPower(leftPower, rightPower);

                //closed
                //score.goToPosGuard(0);
            }else{
                drive.setPower(0.5*leftPower, 0.5*rightPower);

            }


        }

    }

}
