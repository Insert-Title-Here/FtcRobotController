package org.firstinspires.ftc.teamcode.KrishTesting.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.KrishTesting.Command.Talons.TalonsEndGame;
import org.firstinspires.ftc.teamcode.KrishTesting.Command.Talons.TalonsIntake;
import org.firstinspires.ftc.teamcode.KrishTesting.Command.Talons.TalonsScoringSystem;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
public class TestingAuto extends LinearOpMode {

    OpenCvWebcam camera;
    VisionTesting pipeline;
    VisionTesting.Position position;
    TalonsIntake intake;
    TalonsScoringSystem score;
    TalonsEndGame endGame;

    @Override
    public void runOpMode() throws InterruptedException {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new VisionTesting(telemetry);

        camera.setPipeline(pipeline);

        intake = new TalonsIntake(hardwareMap);
        score = new TalonsScoringSystem(hardwareMap);
        endGame = new TalonsEndGame(hardwareMap);



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

        while(!opModeIsActive()){
            position = pipeline.getPosition();
        }






        waitForStart();

        if(position == VisionTesting.Position.RIGHT){
            intake.setPower(0.5);
            sleep(3000);
            intake.brake();
        }else if(position == VisionTesting.Position.MIDDLE){
            score.setRampPower(0.5);
            sleep(3000);
            score.brake();
        }else{
            endGame.runCarouselAuto();
            sleep(3000);
            endGame.brakeCarousel();
        }


    }
}
