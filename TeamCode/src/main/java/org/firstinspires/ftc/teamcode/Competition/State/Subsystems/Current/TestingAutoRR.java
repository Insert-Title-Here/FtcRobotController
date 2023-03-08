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

        /*Trajectory firstTrajectory = drive.trajectoryBuilder(new Pose2d())
                .strafeTo(new Vector2d(-7, -4))
                .build();*/

        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d(), true)
                .splineToConstantHeading(new Vector2d(-7, -4), Math.toRadians(180))
                .splineTo(new Vector2d(-20, -4), Math.toRadians(180))
                .splineTo(new Vector2d(-30, -4), Math.toRadians(180))
                .splineTo(new Vector2d(-37, -4), Math.toRadians(180))
                .splineTo(new Vector2d(-50, -27), Math.toRadians(270))

                                .build();

        Trajectory newestTrajectory = drive.trajectoryBuilder(trajectory.end())
                .strafeRight(5.2)
                        .build();

        Trajectory realign = drive.trajectoryBuilder(newestTrajectory.end())
                .strafeLeft(3.0)
                .build();

        Trajectory across = drive.trajectoryBuilder(realign.end())
                .splineToConstantHeading(new Vector2d(-48, -25), Math.toRadians(270))
                .splineTo(new Vector2d(-48, 80), Math.toRadians(90))
                        .build();

        Trajectory last = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(3.0)
                .build();

        waitForStart();


        drive.followTrajectory(trajectory);
        drive.followTrajectory(newestTrajectory);


        drive.turn(Math.toRadians(9));
        sleep(1000);
        drive.turn(Math.toRadians(0));

        //drive.followTrajectory(realign);
        drive.followTrajectory(across);
        drive.turn(Math.toRadians(189));
    }
}
