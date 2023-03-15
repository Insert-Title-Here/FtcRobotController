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
                //.splineToConstantHeading(new Vector2d(-7, -4), Math.toRadians(180))
                .splineTo(new Vector2d(-20, -0), Math.toRadians(180))
                .splineTo(new Vector2d(-30, -0), Math.toRadians(180))
                .splineTo(new Vector2d(-37, -0), Math.toRadians(180))
                .splineTo(new Vector2d(-50.5, -22), Math.toRadians(270))

                .build();

        TrajectorySequenceBuilder trajectorySequenceBuilder = drive.trajectorySequenceBuilder(new Pose2d())
                .setVelConstraint(new TrajectoryVelocityConstraint() {
                    @Override
                    public double get(double v, @NonNull Pose2d pose2d, @NonNull Pose2d pose2d1, @NonNull Pose2d pose2d2) {
                        return 0;
                    }
                })
                .addTrajectory(trajectory);



        Trajectory newestTrajectory = drive.trajectoryBuilder(trajectory.end())
                .strafeRight(5)
                .build();

        Trajectory realign = drive.trajectoryBuilder(newestTrajectory.end())
                .strafeLeft(4)
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
        wheelieServo.setPosition(Constants.wheelieMedium);

        score.setLinkagePosition(Constants.linkageUpV2);
        //wheelieServo.setPosition(Constants.wheelieMedium);


        score.newLiftPD(57000, 1, 2.0);
        score.setLinkagePosition(Constants.linkageScoreV2);
        sleep(250);
        score.setGrabberPosition(0.3);
        sleep(1000);

        score.setLinkagePositionLogistic(0.26, 500);
        score.newLiftPD(0, 0.6, 2.0);



        sleep(1000);

        while(color.getNormalizedColors().blue < 0.5){
            drive.setMotorPowers(-0.2, -0.2, -0.2, -0.2);
        }

        drive.setMotorPowers(0,0,0,0);


        score.setGrabberPosition(Constants.grabbing);
        sleep(500);

        for(int i = 0; i < 5; i++){
            score.setLinkagePosition(Constants.linkageUpV2);
            sleep(500);
            //wheelieServo.setPosition(Constants.wheelieMedium);


            score.newLiftPD(58500, 1, 2.0);
            score.setLinkagePosition(Constants.linkageScoreV2);
            sleep(250);
            score.setGrabberPosition(0.32);
            sleep(500);

            score.setLinkagePositionLogistic(0.26 - ((i + 1) * 0.03), 500);
            score.newLiftPD(0, 0.6, 2.0);

            sleep(200);

            score.setGrabberPosition(Constants.grabbing);

            sleep(500);


        }



        //wheelieServo.setPosition(Constants.wheelieRetracted);
        //sleep(1000);


        /*drive.turn(Math.toRadians(-4));

        drive.followTrajectory(realign);
        drive.followTrajectory(across);

        drive.followTrajectory(last);
        drive.turn(Math.toRadians(171));

        //drive.followTrajectorySequence(sequence);*/


    }
}
