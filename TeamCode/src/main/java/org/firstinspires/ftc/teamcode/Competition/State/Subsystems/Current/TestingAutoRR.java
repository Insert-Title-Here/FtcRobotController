package org.firstinspires.ftc.teamcode.Competition.State.Subsystems.Current;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Competition.Interleagues.Common.Constants;
import org.firstinspires.ftc.teamcode.Competition.MTI.ScoringSystemNewest;
import org.firstinspires.ftc.teamcode.roadrunnerfiles.drive.DriveConstants;
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
        ColorRangeSensor color = hardwareMap.get(ColorRangeSensor.class, "distance");
        color.setGain(250);


        drive.setPoseEstimate(new Pose2d());

        /*Trajectory firstTrajectory = drive.trajectoryBuilder(new Pose2d())
                .strafeTo(new Vector2d(-7, -4))
                .build();*/




        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d(), true)
                //Need to make the first part deccelerate and the second part go faster
                .splineTo(
                        new Vector2d(-37, -0),
                        Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL + 15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL + 10)
                )
                .splineTo(
                        new Vector2d(-50.5, -22),
                        Math.toRadians(270),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL + 0.4, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )

                .build();



        Trajectory newestTrajectory = drive.trajectoryBuilder(trajectory.end())
                .strafeRight(5,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL + 15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL + 10)
                )
                .build();

        Trajectory realign = drive.trajectoryBuilder(newestTrajectory.end())
                .strafeLeft(4)
                .build();

        Trajectory across = drive.trajectoryBuilder(realign.end())
                .forward(110)
                .build();

        Trajectory last = drive.trajectoryBuilder(across.end(), Math.toRadians(9))
                .strafeRight(5.5)
                .build();

        waitForStart();



        drive.followTrajectory(trajectory);
        drive.followTrajectory(newestTrajectory);


        drive.turn(Math.toRadians(10));
        wheelieServo.setPosition(Constants.wheelieMedium);

        score.setLinkagePosition(Constants.linkageUpV2);
        //wheelieServo.setPosition(Constants.wheelieMedium);


        score.newLiftPD(57000, 1, 1.3);
        score.setLinkagePosition(Constants.linkageScoreV2);
        sleep(70);
        score.setGrabberPosition(0.3);
        sleep(100);

        score.setLinkagePositionLogistic(0.26, 500);
        score.newLiftPD(0, 0.7, 1.2);



        score.setGrabberPosition(Constants.grabbing);
        sleep(50);

        for(int i = 0; i < 5; i++){
            score.setLinkagePosition(Constants.linkageUpV2);
            sleep(150);
            //wheelieServo.setPosition(Constants.wheelieMedium);


            score.newLiftPD(57500, 1, 1.3);
            score.setLinkagePosition(Constants.linkageScoreV2);
            sleep(70);
            score.setGrabberPosition(0.32);
            sleep(100);

            score.setLinkagePositionLogistic(0.26 - ((i + 1) * 0.03), 500);
            score.newLiftPD(0, 0.7, 1.2);

            score.setGrabberPosition(Constants.grabbing);

            sleep(50);


        }






    }
}
