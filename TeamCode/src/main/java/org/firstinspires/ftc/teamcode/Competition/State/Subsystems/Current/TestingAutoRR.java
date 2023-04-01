package org.firstinspires.ftc.teamcode.Competition.State.Subsystems.Current;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Competition.Interleagues.Common.Constants;
import org.firstinspires.ftc.teamcode.Competition.MTI.ScoringSystemNewest;
import org.firstinspires.ftc.teamcode.roadrunnerfiles.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadrunnerfiles.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunnerfiles.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunnerfiles.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

@Autonomous
public class TestingAutoRR extends LinearOpMode {

    //TODO: figure out how to move linkage down while lift goes down
    //TODO: figure out how to turn while moving lift down on the last cycle

    OpenCvWebcam camera;
    KevinGodPipelineAprilTag pipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.resetEncoders();
        MecDriveV2 otherdrive = new MecDriveV2(hardwareMap, false, telemetry, false);
        ScoringSystemNewest score = new ScoringSystemNewest(hardwareMap, telemetry, true);
        Servo wheelieServo = hardwareMap.get(Servo.class, "wheelie");
        Servo cameraOdo = hardwareMap.get(Servo.class, "camera");
        ColorRangeSensor color = hardwareMap.get(ColorRangeSensor.class, "distance");
        color.setGain(250);

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new KevinGodPipelineAprilTag(telemetry, otherdrive, KevinGodPipelineAprilTag.AutoSide.RED_RIGHT, true);

        camera.setPipeline(pipeline);




        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 176, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Init Error", errorCode);
                telemetry.update();

            }
        });

        FtcDashboard.getInstance().startCameraStream(camera, 0);


        cameraOdo.setPosition(0.5);
        Pose2d start = new Pose2d(-34,56,Math.toRadians(90));


        drive.setPoseEstimate(start);

        Trajectory stuff = drive.trajectoryBuilder(start)
                /*.splineToConstantHeading(new Vector2d(-34, 0),
                        Math.toRadians(90),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL + 7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL + 10)
                )*/

                .back(54,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL + 10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL + 7)
                )




                /*.splineTo(new Vector2d(-34, 0),
                        Math.toRadians(270),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL + 3, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL + 5)
                )*/

                .build();

       /* TrajectorySequence first = drive.trajectorySequenceBuilder(stuff.end())
*/

                /*.lineToConstantHeading(new Vector2d(-34, 18),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL + 3, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL + 5)
                )
*/
                /*.addDisplacementMarker(() -> {
                    score.setLinkagePositionLogistic(Constants.linkageUpV2, 500);
                })*/


        TrajectorySequence second = drive.trajectorySequenceBuilder(new Pose2d(-34, 0, Math.toRadians(90)))
                .splineTo(new Vector2d(-55.5, 6),
                        Math.toRadians(180),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL + 12, DriveConstants.MAX_ANG_VEL + 5, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL + 10)
                )
                .splineToLinearHeading(new Pose2d(-57.5, 2.5, Math.toRadians(164)),
                        Math.toRadians(166),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL + 12, DriveConstants.MAX_ANG_VEL + 5, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL + 12)
                        )


                .build();

        Trajectory hopeful = drive.trajectoryBuilder(second.end())
                .splineToLinearHeading(
                new Pose2d(-55, 6, Math.toRadians(0)), Math.toRadians(0),
                SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL + 15, DriveConstants.MAX_ANG_VEL + 5, DriveConstants.TRACK_WIDTH),
                SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL + 8)
                )

                .build();

        TrajectorySequence third = drive.trajectorySequenceBuilder(new Pose2d(-50, 4, 220))
                .forward(100,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL + 22, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL + 22)
                )
                .addTemporalMarker(1, () ->{
                    score.setLinkagePosition(0.28);
                    score.setGrabberPosition(0.45);
                })

                .build();




        Trajectory finalTrajectory = drive.trajectoryBuilder(new Pose2d(0,0,90))
                .strafeRight(4)
                .build();




        /*Trajectory firstTrajectory = drive.trajectoryBuilder(new Pose2d(), true)

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
                        new Vector2d(-51.5, -22.1),
                        Math.toRadians(270),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL + 5, DriveConstants.MAX_ANG_VEL + 5, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL + 5)
                )



                .build();

        Trajectory newestTrajectory = drive.trajectoryBuilder(firstTrajectory.end())
                .strafeRight(5.4,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL + 20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL + 13)
                )
                .build();



        TrajectorySequence stuff2 = drive.trajectorySequenceBuilder(newestTrajectory.end())

                .splineToConstantHeading(
                        new Vector2d(-50, -23.5), Math.toRadians(-9),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL + 20, DriveConstants.MAX_ANG_VEL + 5, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL + 13)
                )

                .splineToLinearHeading(
                        new Pose2d(-45.5, -21, Math.toRadians(260)),
                        Math.toRadians(260),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL + 20, DriveConstants.MAX_ANG_VEL + 7, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL + 10)
                )

                .addDisplacementMarker(() -> {
                    score.setLinkagePosition(0.28);
                })

                .addTemporalMarker(2, () -> {
                    wheelieServo.setPosition(Constants.wheelieHigh);
                })

                *//*
                .back(75,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL + 30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL + 20)
                )

                .back(30,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL + 15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL + 10)
                )
                *//*

               .lineToConstantHeading(new Vector2d(-30.5, 91.5),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL + 30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL + 20)
                )

                *//*.splineTo(new Vector2d(-48, 70),
                        Math.toRadians(80),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL + 20)
                )*//*






                //.lineToLinearHeading(new Pose2d(-51, 80, Math.toRadians(90)))
                .build();


        Trajectory finalstrafe = drive.trajectoryBuilder(stuff2.end())
                .strafeLeft(5.4,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL + 20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL + 13)
                )
                .build();

*/
        pipeline.changeMode(KevinGodPipelineAprilTag.Mode.REDCONE);
        waitForStart();

        drive.followTrajectory(stuff);
        drive.setPoseEstimate(new Pose2d(-34, 0, Math.toRadians(90)));
        drive.followTrajectorySequence(second);
        wheelieServo.setPosition(0.4);


        //score.newLiftPD(60000, 1, 1.2);
        score.moveToPosition(60000, 1, 1);
        score.setLinkagePosition(Constants.linkageScoreV2 + 0.03);
        sleep(75);
        score.setGrabberPosition(0.5);
        sleep(150);
        score.setLinkagePosition(0.28);
        sleep(225);
        score.newLiftPD(2000, 0.55, 0.75);
        score.setGrabberPosition(Constants.grabbing);
        sleep(75);


        for (int i = 0; i < 5; i++) {
            score.setLinkagePosition(Constants.linkageUpV2 + 0.1);
            sleep(150);



            //score.newLiftPD(60000, 1, 1.2);
            score.moveToPosition(60000, 1, 1);
            score.setLinkagePosition(Constants.linkageScoreV2 + 0.03);
            sleep(75);
            score.setGrabberPosition(0.5);
            sleep(150);
            if (i == 0) {
                score.setLinkagePosition(0.25);
            } else if(i == 1){
                score.setLinkagePosition(0.22);
            }else if(i == 2){
                score.setLinkagePosition(0.17);
            }else if(i == 3){
                score.setLinkagePosition(0.14);

            }else{
                score.setLinkagePosition(Constants.linkageUpV2);
            }
            sleep(225);

            if(i == 0){
                score.newLiftPD(1500, 0.6, 0.7);
                score.setGrabberPosition(Constants.grabbing);
                sleep(110);
            }else {
                score.newLiftPD(0, 0.6, 0.7);
                score.setGrabberPosition(Constants.grabbing);
                sleep(80);
            }




        }

        wheelieServo.setPosition(Constants.wheelieRetracted);

        drive.followTrajectory(hopeful);
        otherdrive.tankRotateSpecial(-1.61, 0.2);

        //Move Across


        //-1.5 is too far to the pole
        //-1.46 is too far the other way



        /*otherdrive.tankRotateSpecial(-1, 0.6);



        while(Math.abs(-1.48 - otherdrive.getFirstAngle()) > 0.005){
            drive.setMotorPowers(0.4, 0.4, 0.3, 0.3);
        }
        drive.setMotorPowers(0,0,0,0);
        sleep(2000);
*/

        score.setGrabberPosition(0.49);
        score.setLinkagePosition(0.28);
        drive.resetEncoders();
        drive.pid(93, 4, 3);




        //otherdrive.setPowerAutoSpecial(0.2, MecDriveV2.MovementType.ROTATE);

        //score.setGrabberPosition(Constants.openV2);
        //drive.followTrajectorySequence(third);

        // put camera stuff

        pipeline.normalizeStrafeSpecial(-0.25, 165,2);

        while(color.getNormalizedColors().red < 0.6){
            drive.setMotorPowers(0.4, 0.4, 0.4, 0.4);
        }
        drive.setMotorPowers(0, 0, 0, 0);

        score.setGrabberPosition(Constants.grabbing);

        sleep(150);

        score.setLinkagePosition(Constants.linkageUpV2);

        sleep(150);





        //wheelieServo.setPosition(0.4);

        otherdrive.tankRotatePIDMoreSpecial(-1.2, 0.5, false, 1);

        drive.setMotorPowers(-0.45,-0.45,-0.45,-0.45);
        sleep(350);
        drive.setMotorPowers(0,0,0,0);

        score.newLiftPD(61000, 1, 1.2);
        score.setLinkagePosition(Constants.linkageScoreV2 + 0.03);
        sleep(225);
        score.setGrabberPosition(0.47);
        sleep(75);

        score.setLinkagePosition(Constants.linkageUpV2);
        sleep(75);

        score.newLiftPD(0, 0.55, 0.75);


        score.setLinkagePosition(Constants.linkageScoreV2);

        drive.setMotorPowers(0.65,0.65,0.2,0.2);
        sleep(500);
        drive.setMotorPowers(0,0,0,0);
        sleep(50);



    }






}
