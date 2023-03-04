package org.firstinspires.ftc.teamcode.Competition.State.Subsystems.Current;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunnerfiles.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunnerfiles.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous
public class TestingAutoRR extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d());
        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d(), true)
                        .splineTo(new Vector2d(-45, 0), Math.toRadians(270))
                                .build();

        waitForStart();

        drive.followTrajectory(trajectory);

    }
}
