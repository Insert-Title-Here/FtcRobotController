package org.firstinspires.ftc.teamcode.Autonomous.WIP;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Common.Constants;
import org.firstinspires.ftc.teamcode.Common.MecDriveV2;
import org.firstinspires.ftc.teamcode.Common.ScoringSystem;
@Disabled
@Autonomous
public class BlueFrontAutoTest extends LinearOpMode {

    MecDriveV2 drive;
    ScoringSystem score;
    ElapsedTime time;

    public void runOpMode() {

        time = new ElapsedTime();
        drive = new MecDriveV2(hardwareMap, false, telemetry, time);
        score = new ScoringSystem(hardwareMap, telemetry, time);
        score.setGrabberPosition(Constants.OPEN);
        score.setGrabberPosition(Constants.GRABBING);

        waitForStart();
        drive.simpleMoveToPosition(615, 0.4);
        score.setIntakePower(-0.2);
        sleep(500);
        score.setIntakePower(0);
        drive.simpleMoveToPosition(-420, 0.3);

        drive.simpleMoveToPosition(-650, MecDriveV2.MovementType.STRAFE, 0.3);

        drive.tankRotate((Math.PI/(-2)), 0.3);

        drive.simpleBrake();

        sleep(500);

    }

}
