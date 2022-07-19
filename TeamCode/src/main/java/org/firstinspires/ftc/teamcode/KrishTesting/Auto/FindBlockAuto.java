package org.firstinspires.ftc.teamcode.KrishTesting.Auto;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.KrishTesting.Command.Talons.RobotT;
import org.firstinspires.ftc.teamcode.KrishTesting.Command.Talons.TalonsEndGame;
import org.firstinspires.ftc.teamcode.KrishTesting.Command.Talons.TalonsIntake;
import org.firstinspires.ftc.teamcode.KrishTesting.Command.Talons.TalonsScoringSystem;
import org.firstinspires.ftc.teamcode.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Vector2D;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.io.FileNotFoundException;

@Autonomous
public class FindBlockAuto extends LinearOpMode {


    RobotT robot;
    OpenCvWebcam camera;
    FindBlockPipeline pipeline;
    TalonsIntake intake;
    TalonsScoringSystem score;
    TalonsEndGame endGame;
    MecanumDriveTrain drive;

    Thread updateThread;

    @Override
    public void runOpMode() throws InterruptedException {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new FindBlockPipeline(telemetry);

        camera.setPipeline(pipeline);

        intake = new TalonsIntake(hardwareMap);
        score = new TalonsScoringSystem(hardwareMap);
        endGame = new TalonsEndGame(hardwareMap);

        try {
            drive = new MecanumDriveTrain(hardwareMap);
            robot = new RobotT(hardwareMap);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }


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

        updateThread = new Thread(){
            @Override
            public void run(){
                while(opModeIsActive()){
                    robot.update();
                }


            }
        };


        waitForStart();

        updateThread.start();

        for(int i = 0; i < 3; i++) {

            while (!pipeline.getInMiddle()) {
                drive.setPower(-0.4, 0.4, -0.4, 0.4);
            }
            drive.brake();

            while (robot.color.getDistance(DistanceUnit.INCH) > 1) {
                drive.setPower(0.3, 0.3, 0.3, 0.3);
                intake.setPower(1);
            }
            score.close();
            drive.brake();
            intake.brake();

            sleep(500);
            score.up();
            sleep(600);
            score.score();
            score.setRampPower(1.0);
            sleep(3000);
            score.brake();
            sleep(200);
            score.down();
            score.open();
            pipeline.inMiddle = false;
        }



    }
}
