package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args){
        MeepMeep meepMeep = new MeepMeep(800);




        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 13.51)

                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-34,56,270))
                        .splineTo(new Vector2d(-34, 40), Math.toRadians(270))
                        .splineTo(new Vector2d(-34, 12), Math.toRadians(270))
                        .splineTo(new Vector2d(-50, 4), Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(-55, -2, Math.toRadians(170)), Math.toRadians(170))

                        .splineToLinearHeading(new Pose2d(-55, 4, Math.toRadians(10)), Math.toRadians(10))
                        .lineTo(new Vector2d(50, 25))
                        .splineToLinearHeading(new Pose2d(52, 20, Math.toRadians(0)), Math.toRadians(0))
                                .build());







        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}