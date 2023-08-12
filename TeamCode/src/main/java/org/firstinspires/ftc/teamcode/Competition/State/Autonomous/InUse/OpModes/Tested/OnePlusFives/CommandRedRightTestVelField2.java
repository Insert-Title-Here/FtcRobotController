package org.firstinspires.ftc.teamcode.Competition.State.Autonomous.InUse.OpModes.Tested.OnePlusFives;

//import com.acmerobotics.dashboard.FtcDashboard;

//Changed camera pole height
//Checked pipeline parameter


import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Competition.Interleagues.Common.Constants;
import org.firstinspires.ftc.teamcode.Competition.State.Subsystems.Current.KevinGodPipelineAprilTag;
import org.firstinspires.ftc.teamcode.Competition.State.Subsystems.Current.MecDriveV2;
import org.firstinspires.ftc.teamcode.Competition.State.Subsystems.Current.ScoringSystemV2EpicLift;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.atomic.AtomicBoolean;

@Autonomous(name = "Red Right Field 2")
public class CommandRedRightTestVelField2 extends LinearOpMode {
    MecDriveV2 drive;
    ScoringSystemV2EpicLift score;
    //Constants constants;
    ElapsedTime time = new ElapsedTime();
    AtomicBoolean hold, armUp, armDown, finalMove, linkageUp;
    int cycles;
    int counter = 0;

    ColorRangeSensor distance;
    Servo cameraServo;

    OpenCvWebcam camera;
    KevinGodPipelineAprilTag pipeline;
    KevinGodPipelineAprilTag.ParkPos parkPos;

    boolean failed;
    boolean preloadSuccess = false;


    @Override
    public void runOpMode() throws InterruptedException {
        distance = hardwareMap.get(ColorRangeSensor.class, "distance");

        drive = new MecDriveV2(hardwareMap, false, telemetry, true);
        //constants = newConstants();
        score = new ScoringSystemV2EpicLift(hardwareMap, telemetry, false);
        hold = new AtomicBoolean(false);
        armUp = new AtomicBoolean(false);
        finalMove = new AtomicBoolean(false);
        linkageUp = new AtomicBoolean(false);
        armDown = new AtomicBoolean(false);

        score.setLinkagePosition(Constants.linkageScoreV2 - 0.07);
        score.setGrabberPosition(Constants.grabbing);


        cameraServo = hardwareMap.get(Servo.class, "camera");
        failed = false;


        distance.setGain(300);

        cycles = 5;


        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new KevinGodPipelineAprilTag(telemetry, drive, KevinGodPipelineAprilTag.AutoSide.RED_RIGHT, false);

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



        cameraServo.setPosition(Constants.sleeveV2);


        //FtcDashboard.getInstance().startCameraStream(camera, 0);


        waitForStart();

        double startTime = time.seconds();


        CommandScheduler.getInstance().schedule(
                new InstantCommand(() -> pipeline.changeMode(KevinGodPipelineAprilTag.Mode.REDCONE)),
                new InstantCommand(() -> cameraServo.setPosition(Constants.coneV2)),
                new InstantCommand(() -> parkPos = pipeline.getPosition()),
                new InstantCommand(() -> drive.goTOPIDPosVel(-2050, 500)),
                new WaitCommand(100),
                new InstantCommand(() -> drive.tankRotatePID(Math.PI / 2, 1, false)),
                //new InstantCommand(() -> drive.goTOPIDPosVel(790)),
                new InstantCommand(() -> drive.simpleMoveToPosition(695, MecDriveV2.MovementType.STRAIGHT, 0.5)),

                new InstantCommand(() -> drive.tankRotatePID(3.14 * Math.PI / 8, 1, false)),
                new InstantCommand(() -> pipeline.normalizeStrafe(0.3, 150, 2)),
                new InstantCommand(() -> pipeline.changeMode(KevinGodPipelineAprilTag.Mode.POLE)),
                new InstantCommand(() -> cameraServo.setPosition(0.375)),
                new WaitCommand(500),
                //new InstantCommand(() -> pipeline.normalize(0.22, 162, 3, /*(Math.PI/2) - (3 * Math.PI/45)*/1.269, /*(Math.PI/2) - (Math.PI/18)*/1.433, /*(Math.PI/2) - (13 * Math.PI/180)*/1.355))
                new InstantCommand(() -> drive.tankRotatePIDSpecial(1.355, 0.4, false, 1))

        );

        if(distance.getNormalizedColors().red > 0.6) {
            CommandScheduler.getInstance().schedule(

                    //TODO: need to add preload logic (whether preload or not)
                    new ParallelCommandGroup(
                            new InstantCommand(() -> score.setLinkagePositionLogistic(Constants.linkageUpV2Auto, 300, 100)),
                            new InstantCommand(() -> score.newLiftPID(960, 0.95, 1))
                    ),

                    new InstantCommand(() -> score.setLinkagePositionLogistic(0.8, 100)),
                    new WaitCommand(200),
                    new InstantCommand(() -> score.setGrabberPosition(Constants.score + 0.1)),
                    new WaitCommand(400),
                    new InstantCommand(() -> sleep(400)),
                    new InstantCommand(() -> score.setLinkagePositionLogistic(0.245, 100)),
                    new InstantCommand(() -> score.setGrabberPosition(Constants.openV2 - 0.05)),

                    new WaitCommand(200),

                    new InstantCommand(() -> score.moveToPosition(0, 0.63))



            );
        }else {
            CommandScheduler.getInstance().schedule(
                    new InstantCommand(() -> score.setLinkagePositionLogistic(0.245, 100)),
                    new InstantCommand(() -> score.setGrabberPosition(Constants.openV2 + 0.1))
            );
        }


        CommandScheduler.getInstance().schedule(

                new InstantCommand(() -> drive.simpleMoveToPosition(75, MecDriveV2.MovementType.STRAIGHT, 0.5)),
                new InstantCommand(() -> score.setGrabberPosition(Constants.grabbing)),
                new WaitCommand(100),
                new InstantCommand(() -> score.setLinkagePositionLogistic(Constants.linkageUpV2Auto, 300, 100)),
                //new InstantCommand(() -> pipeline.normalize(0.22, 169, 3, /*(Math.PI/2) - (3 * Math.PI/45)*/1.269, /*(Math.PI/2) - (Math.PI/18)*/1.433, /*(Math.PI/2) - (13 * Math.PI/180)*/1.355)),
                new InstantCommand(() -> score.newLiftPID(1020, 0.95, 1)),


                new InstantCommand(() -> score.setLinkagePositionLogistic(0.8, 100)),
                new WaitCommand(100),
                new InstantCommand(() -> score.setGrabberPosition(Constants.score + 0.1)),
                new WaitCommand(400),
                new InstantCommand(() -> score.setLinkagePositionLogistic(0.242, 800, 100)),


                new InstantCommand(() -> score.setGrabberPosition(Constants.openV2 - 0.1)),
                new InstantCommand(() -> score.moveToPosition(0, 0.63))

        );

        //Should add 5 cycles
        for(int i = 0; i < 4; i++){
            int finalI = i;

            //TODO: ask kevin what this is for
            /*if (i != 3) {
                finalI = i;
            } else {
                finalI = 2;
            }*/

            CommandScheduler.getInstance().schedule(
                    new InstantCommand(() -> drive.simpleMoveToPosition(8, MecDriveV2.MovementType.STRAIGHT, 0.5)),
                    new InstantCommand(() -> score.setGrabberPosition(Constants.grabbing)),
                    new WaitCommand(100),
                    new InstantCommand(() -> score.setLinkagePositionLogistic(Constants.linkageUpV2Auto, 300, 100))

            );

            if(counter > 1){
                break;
            }

            if(distance.getNormalizedColors().red < 0.7){
                CommandScheduler.getInstance().schedule(
                        new InstantCommand(() -> score.setGrabberPosition(Constants.openV2- 0.1)),
                        new InstantCommand(() -> score.setLinkagePositionLogistic(0.242 - ((finalI) * 0.03), 800, 100)),
                        new InstantCommand(() -> drive.simpleMoveToPosition(8, MecDriveV2.MovementType.STRAIGHT, 0.5)),
                        new InstantCommand(() -> score.setGrabberPosition(Constants.grabbing)),
                        new WaitCommand(100),
                        new InstantCommand(() -> score.setLinkagePositionLogistic(Constants.linkageUpV2Auto, 300, 100))
                );

                counter++;



            }


            CommandScheduler.getInstance().schedule(


                    //new InstantCommand(() -> pipeline.normalize(0.22, 169, 3, /*(Math.PI/2) - (3 * Math.PI/45)*/1.269, /*(Math.PI/2) - (Math.PI/18)*/1.433, /*(Math.PI/2) - (13 * Math.PI/180)*/1.355)),
                    new InstantCommand(() -> score.newLiftPID(1012, 0.95, 1))
            );

            if(distance.getNormalizedColors().red < 0.7){
                CommandScheduler.getInstance().schedule(
                        new InstantCommand(() -> score.setLinkagePosition(Constants.linkageUpV2)),
                        new InstantCommand(() -> score.moveToPosition(0, 0.63))
                );

                break;
            }

            CommandScheduler.getInstance().schedule(


                    new InstantCommand(() -> score.setLinkagePositionLogistic(0.8, 100)),
                    new WaitCommand(100),
                    new InstantCommand(() -> score.setGrabberPosition(Constants.score + 0.1)),
                    new WaitCommand(400),
                    new InstantCommand(() -> score.setLinkagePositionLogistic(0.242 - ((finalI + 1) * 0.03), 800, 100)),


                    new InstantCommand(() -> score.setGrabberPosition(Constants.openV2- 0.1)),
                    new InstantCommand(() -> score.moveToPosition(0, 0.63))
            );


            //TODO: added this
            // Check for cone and cycle again if one is present
            if (i == 3 && distance.getNormalizedColors().red > 0.6) {
                i = 2;
            }

            if (time.seconds() - startTime > 25) {
                i = 5;
            }
        }


        CommandScheduler.getInstance().schedule(
                new InstantCommand(() -> drive.simpleMoveToPosition(-80, MecDriveV2.MovementType.ROTATE, 0.5)),
                new WaitCommand(50)
        );


        CommandScheduler.getInstance().run();

        while(CommandScheduler.getInstance().isScheduled(
                new InstantCommand(() -> drive.tankRotatePIDSpecial(Math.PI/2, 0.4, false, 1)),
                new InstantCommand(() -> sleep(50))
        )){

        }

        if (parkPos == KevinGodPipelineAprilTag.ParkPos.CENTER) {
            drive.simpleMoveToPosition(-700, MecDriveV2.MovementType.STRAIGHT, 1);
        } else if (parkPos == KevinGodPipelineAprilTag.ParkPos.LEFT) {
            drive.simpleMoveToPosition(-1450, MecDriveV2.MovementType.STRAIGHT, 1);
        }

        score.setGrabberPosition(Constants.grabbing);

        sleep(200);

        drive.tankRotatePID(0, 1, false);
        drive.simpleMoveToPosition(250, MecDriveV2.MovementType.STRAIGHT, 1);
        sleep(50);







    }
}
