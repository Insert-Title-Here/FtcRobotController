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

    //TODO: figure out how to move linkage down while lift goes down
    //TODO: figure out how to turn while moving lift down on the last cycle

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ScoringSystemNewest score = new ScoringSystemNewest(hardwareMap, telemetry, true);
        Servo wheelieServo = hardwareMap.get(Servo.class, "wheelie");
        ColorRangeSensor color = hardwareMap.get(ColorRangeSensor.class, "distance");
        color.setGain(250);


        drive.setPoseEstimate(new Pose2d());

        Trajectory firstTrajectory = drive.trajectoryBuilder(new Pose2d(), true)

                .splineTo(
                        new Vector2d(-15, -0),
                        Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL + 17, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL + 15)
                )

                .splineTo(
                        new Vector2d(-29, -0),
                        Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL + 5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL + 10)
                )

                .splineTo(
                        new Vector2d(-38, -0),
                        Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL + 5)
                )

                .splineTo(
                        new Vector2d(-51, -22),
                        Math.toRadians(270),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL + 5, DriveConstants.MAX_ANG_VEL + 5, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL + 5)
                )
                .build();








        Trajectory newestTrajectory = drive.trajectoryBuilder(firstTrajectory.end())
                .strafeRight(5,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL + 20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL + 13)
                )
                .build();

        Trajectory realign = drive.trajectoryBuilder(newestTrajectory.end())
                .strafeLeft(4,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL + 20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL + 13)
                )
                .build();

        Trajectory across = drive.trajectoryBuilder(realign.end(), true)
                .back(110,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL + 25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL + 17)
                )
                .build();

        Trajectory last = drive.trajectoryBuilder(across.end(), Math.toRadians(9))
                .strafeRight(5.5,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL + 20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL + 13)
                )
                .build();

        waitForStart();


        drive.followTrajectory(firstTrajectory);
        drive.followTrajectory(newestTrajectory);


        drive.turn(Math.toRadians(10));
        wheelieServo.setPosition(Constants.wheelieMedium);

        score.setLinkagePosition(Constants.linkageUpV2 + 0.1);
        //wheelieServo.setPosition(Constants.wheelieMedium);


        score.newLiftPD(57500, 1, 0.95);
        score.setLinkagePosition(Constants.linkageScoreV2);
        sleep(70);
        score.setGrabberPosition(0.3);
        sleep(100);

        score.setLinkagePositionLogistic(0.26, 500);
        score.newLiftPD(0, 0.75, 0.75);



        score.setGrabberPosition(Constants.grabbing);
        sleep(50);

        for(int i = 0; i < 5; i++){
            score.setLinkagePosition(Constants.linkageUpV2 + 0.1);
            sleep(150);
            //wheelieServo.setPosition(Constants.wheelieMedium);


            score.newLiftPD(57500, 1, 0.95);
            score.setLinkagePosition(Constants.linkageScoreV2);
            sleep(70);
            score.setGrabberPosition(0.32);
            sleep(100);
            if(i < 4) {
                score.setLinkagePositionLogistic(0.26 - ((i + 1) * 0.03), 500);
            }else{
                score.setLinkagePositionLogistic(0.14, 500);
            }
            score.newLiftPD(0, 0.75, 0.75);

            score.setGrabberPosition(Constants.grabbing);

            sleep(50);


        }

        drive.turn(Math.toRadians(-4));
        drive.followTrajectory(realign);
        drive.turn(Math.toRadians(180));
        drive.followTrajectory(across);
        //drive.followTrajectory(last);

        drive.turn(Math.toRadians(165));






    }
}
