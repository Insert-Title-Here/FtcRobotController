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

        Trajectory firstTrajectory = drive.trajectoryBuilder(new Pose2d())
                .strafeTo(new Vector2d(-7, -4))
                .build();

        Trajectory trajectory = drive.trajectoryBuilder(firstTrajectory.end(), true)

                .splineTo(new Vector2d(-20, -4), Math.toRadians(180))
                .splineTo(new Vector2d(-30, -4), Math.toRadians(180))
                .splineTo(new Vector2d(-50, -27), Math.toRadians(270))

                                .build();

        Trajectory newestTrajectory = drive.trajectoryBuilder(trajectory.end())
                .strafeRight(5.2)
                        .build();

        waitForStart();

        drive.followTrajectory(firstTrajectory);
        drive.followTrajectory(trajectory);
        drive.followTrajectory(newestTrajectory);

        drive.turn(Math.toRadians(9));

    }
}
