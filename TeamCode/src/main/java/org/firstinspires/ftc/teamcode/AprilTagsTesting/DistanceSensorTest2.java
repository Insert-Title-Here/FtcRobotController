package org.firstinspires.ftc.teamcode.AprilTagsTesting;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.League1.Common.Constants;
import org.firstinspires.ftc.teamcode.V2.NewSubsystem.ScoringSystemV2EpicLift;

@TeleOp
public class DistanceSensorTest2 extends LinearOpMode {
    boolean hasSeenPole = false;

    DistanceSensor distance;
    ScoringSystemV2EpicLift score;
    Constants constants;


    @Override
    public void runOpMode() throws InterruptedException {
        distance = hardwareMap.get(DistanceSensor.class, "DistancePole");

        constants = new Constants();
        score = new ScoringSystemV2EpicLift(hardwareMap, constants, telemetry);
        score.setLinkagePosition(0.72);

        waitForStart();

        score.setGrabberPosition(Constants.grabbing);

        sleep(5000);

        score.moveToPosition(800, 1);
        int position = score.moveToPosition(1100, 0.5, 2, true);
        score.setPower(0);

        sleep(1000);

        score.setGrabberPosition(Constants.openV2);

        while (opModeIsActive()) {
            telemetry.addData("distance", distance.getDistance(DistanceUnit.CM));
            telemetry.addData("actual position", score.getLeftEncoderPos());
            telemetry.addData("end position", position);
            telemetry.update();
            if (distance.getDistance((DistanceUnit.CM)) < 30 && !hasSeenPole) {
                score.setGrabberPosition(Constants.openV2);
            }
        }


    }


}
