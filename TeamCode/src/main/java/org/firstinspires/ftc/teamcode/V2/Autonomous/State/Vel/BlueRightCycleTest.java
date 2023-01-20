package org.firstinspires.ftc.teamcode.V2.Autonomous.State.Vel;

import com.acmerobotics.dashboard.FtcDashboard;
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
import org.firstinspires.ftc.teamcode.AprilTagsTesting.KevinGodPipelineAprilTag;
import org.firstinspires.ftc.teamcode.League1.Common.Constants;
import org.firstinspires.ftc.teamcode.V2.NewSubsystem.MecDriveV2;
import org.firstinspires.ftc.teamcode.V2.NewSubsystem.ScoringSystemV2EpicLift;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.atomic.AtomicBoolean;

@Autonomous (name = "Blue Right Cycle Test")
public class BlueRightCycleTest extends LinearOpMode {
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
        pipeline = new KevinGodPipelineAprilTag(telemetry, drive, KevinGodPipelineAprilTag.AutoSide.BLUE_RIGHT, true);

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


        pipeline.changeMode(KevinGodPipelineAprilTag.Mode.POLE);
        cameraServo.setPosition(Constants.poleV2);


        waitForStart();

        double startTime = time.seconds();

        parkPos = KevinGodPipelineAprilTag.ParkPos.LEFT;


        //Should add 5 cycles
        for(int i = 0; i < 5; i++){
            telemetry.addData("stuff", i);
            int finalI = i;
            CommandScheduler.getInstance().schedule(
                    new InstantCommand(() -> drive.simpleMoveToPosition(8, MecDriveV2.MovementType.STRAIGHT, 0.5)),
                    new InstantCommand(() -> score.setGrabberPosition(Constants.grabbing)),
                    new WaitCommand(100),
                    new InstantCommand(() -> score.setLinkagePositionLogistic(Constants.linkageUpV2Auto, 300, 100))
            );

            if(counter > 1){
                break;
            }

            if(distance.getNormalizedColors().blue < 0.7){
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
                    new InstantCommand(() -> pipeline.normalize(0.22, 169, 3)),

                    //new InstantCommand(() -> pipeline.normalize(0.15, 169, 3)),
                    new InstantCommand(() -> score.newLiftPID(1012, 1, 0.83))
            );


            if(distance.getNormalizedColors().blue < 0.7){
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

            // Check for cone and cycle again if one is present
            if (i == 3 && distance.getNormalizedColors().blue > 0.4) {
                i = 2;
            }

            if (time.seconds() - startTime > 25) {
                i = 5;
            }

            telemetry.update();
        }


        CommandScheduler.getInstance().schedule(
                new InstantCommand(() -> drive.simpleMoveToPosition(-80, MecDriveV2.MovementType.ROTATE, 0.5)),
                new WaitCommand(50)
        );


        CommandScheduler.getInstance().run();

        while(CommandScheduler.getInstance().isScheduled(
                new InstantCommand(() -> drive.simpleMoveToPosition(-80, MecDriveV2.MovementType.ROTATE, 0.5)),
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

        drive.tankRotatePID(-Math.PI/2, 1, false);
        drive.simpleMoveToPosition(250, MecDriveV2.MovementType.STRAIGHT, 1);
        sleep(50);



    }
}
