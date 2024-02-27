package org.firstinspires.ftc.teamcode.Kids;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Common.Constants;
import org.firstinspires.ftc.teamcode.Common.MecDriveV2;
import org.firstinspires.ftc.teamcode.Common.ScoringSystem;
import org.firstinspires.ftc.teamcode.Common.Vector2D;

@Disabled
@TeleOp
public class KidsTeleOpTank extends LinearOpMode {

    MecDriveV2 drive;
    ElapsedTime time;
    public void runOpMode() {

        time = new ElapsedTime();
        drive = new MecDriveV2(hardwareMap, false, telemetry, time);
        waitForStart();

        while(opModeIsActive()) {
            double rotatePow = gamepad1.left_stick_x;
            double drivePow = gamepad1.left_stick_y * -1;
            double leftPower = Range.clip(drivePow - rotatePow, -1, 1);
            double rightPower = Range.clip(drivePow + rotatePow, -1, 1);
            drive.setPower(leftPower, rightPower, leftPower, rightPower);


        }

    }

}
