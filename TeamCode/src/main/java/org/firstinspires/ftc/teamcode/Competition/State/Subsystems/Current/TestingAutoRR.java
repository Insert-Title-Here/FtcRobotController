package org.firstinspires.ftc.teamcode.Competition.State.Subsystems.Current;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Competition.Interleagues.Common.Constants;
import org.firstinspires.ftc.teamcode.Competition.MTI.ScoringSystemNewest;
import org.firstinspires.ftc.teamcode.roadrunnerfiles.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunnerfiles.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunnerfiles.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous
public class TestingAutoRR extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ScoringSystemNewest score = new ScoringSystemNewest(hardwareMap, telemetry, true);
        Servo wheelieServo = hardwareMap.get(Servo.class, "wheelie");



        drive.setPoseEstimate(new Pose2d());

        /*Trajectory firstTrajectory = drive.trajectoryBuilder(new Pose2d())
                .strafeTo(new Vector2d(-7, -4))
                .build();*/

        TrajectorySequence sequence = drive.trajectorySequenceBuilder(new Pose2d())

                .lineToLinearHeading(new Pose2d(10, 0, Math.toRadians(90)))
                .waitSeconds(1)
                /*.splineTo(new Vector2d(-20, -4), Math.toRadians(180))
                .waitSeconds(1)
                .splineTo(new Vector2d(-30, -4), Math.toRadians(180))
                .waitSeconds(1)
                .splineTo(new Vector2d(-37, -4), Math.toRadians(180))
                .waitSeconds(1)
                .splineTo(new Vector2d(-51.5, -26), Math.toRadians(270))
                .waitSeconds(1)
                .strafeRight(5)
                .turn(Math.toRadians(9))
                .waitSeconds(3)
                .turn(Math.toRadians(-4))
                .strafeLeft(5.2)
                .forward(110)
                .strafeRight(4.0)
                .turn(171)

                 */
                .build();



        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d(), true)
                .splineToConstantHeading(new Vector2d(-7, -4), Math.toRadians(180))
                .splineTo(new Vector2d(-20, -4), Math.toRadians(180))
                .splineTo(new Vector2d(-30, -4), Math.toRadians(180))
                .splineTo(new Vector2d(-37, -4), Math.toRadians(180))
                .splineTo(new Vector2d(-51.5, -27), Math.toRadians(270))

                                .build();

        Trajectory newestTrajectory = drive.trajectoryBuilder(trajectory.end())
                .strafeRight(5)
                        .build();

        Trajectory realign = drive.trajectoryBuilder(newestTrajectory.end())
                .strafeLeft(5.2)
                .build();

        Trajectory across = drive.trajectoryBuilder(realign.end())
                .forward(110)
                        .build();

        Trajectory last = drive.trajectoryBuilder(across.end(), Math.toRadians(9))
                .strafeRight(4.0)
                .build();

        waitForStart();


        drive.followTrajectory(trajectory);
        drive.followTrajectory(newestTrajectory);


        drive.turn(Math.toRadians(10));

        score.setLinkagePosition(Constants.linkageUpV2);
        //wheelieServo.setPosition(Constants.wheelieMedium);
        score.newLiftPD(56000, 1, 2.0);
        score.setLinkagePosition(Constants.linkageScoreV2);
        sleep(250);
        score.setGrabberPosition(Constants.score);
        sleep(1000);
        score.setLinkagePositionLogistic(Constants.linkageUpV2, 500);
        score.newLiftPD(0, 0.5, 2.0);
        score.setLinkagePositionLogistic(Constants.linkageDownV2, 500);

        sleep(1000);

        wheelieServo.setPosition(Constants.wheelieRetracted);
        sleep(1000);


        /*drive.turn(Math.toRadians(-4));

        drive.followTrajectory(realign);
        drive.followTrajectory(across);

        drive.followTrajectory(last);
        drive.turn(Math.toRadians(171));

        //drive.followTrajectorySequence(sequence);*/



    }
}
