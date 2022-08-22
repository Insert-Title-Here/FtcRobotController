package org.firstinspires.ftc.teamcode.Testing.FtcDashboard;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Vector2D;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.io.FileNotFoundException;

@Config
@TeleOp(name="FTC Dashboard Test")
public class FtcDashboardTest extends LinearOpMode {

    Thread driveThread;
    Thread intakeThread;
    MecanumDriveTrain drive;
    //Intake intake;
    ColorSensor color;

    TelemetryPacket testPacket = new TelemetryPacket();
    ElapsedTime timer = new ElapsedTime();

    //Vision Constants
    public static int R1X = 70;
    public static int R1Y = 190;
    public static int R2X = 280;
    public static int R2Y = 190;
    public static int WIDTH = 50;
    public static int HEIGHT = 40;

    WebcamName wc;
    OpenCvCamera camera;

    static final BarcodePipelineBlue brp = new BarcodePipelineBlue();



    boolean auto = false;
    boolean changePower = true;
    private double power = 1;
    private final double NORMAL_LINEAR_MODIFIER = 0.85;
    private final double NORMAL_ROTATIONAL_MODIFIER = 0.45;
    private final double SPRINT_LINEAR_MODIFIER = 1;
    private final double SPRINT_ROTATIONAL_MODIFIER = 0.75;

    @Override
    public void runOpMode() throws InterruptedException{

        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.updateConfig();

        //color = hardwareMap.get(ColorSensor.class, "color");


        try {
            drive = new MecanumDriveTrain(hardwareMap);
            //intake = new Intake(hardwareMap);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }



        driveThread = new Thread(){
            @Override
            public void run(){
                while(opModeIsActive()){
                    driveUpdate();
                }
            }
        };

        intakeThread = new Thread(){
            @Override
            public void run(){
                while(opModeIsActive()){
                    //intakeUpdate();
                }
            }
        };

        //color.enableLed(true);

        testPacket.put("Status", "Initializing...");
        testPacket.put("Elapsed Time", "-");
        dashboard.sendTelemetryPacket(testPacket);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        wc = hardwareMap.get(WebcamName.class, "Webcam");

        // W/ or W/ out live preview
        camera = OpenCvCameraFactory.getInstance().createWebcam(wc, cameraMonitorViewId);
        // camera = OpenCvCameraFactory.getInstance().createWebcam(wc);

        camera.setPipeline(brp);

        // Open an asynchronous connection to the device
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            // Start opening the camera and stream it
            @Override
            public void onOpened() {

                /*
                // create a rgb2gray mat pipeline
                class GrayPipeline extends OpenCvPipeline {
                    Mat gray = new Mat();
                    @Override
                    public Mat processFrame(Mat input) {
                        // mat src, mat dst, int code, convert rgb img to gray
                        Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY);
                        return gray;
                    }
                } */

                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(camera, 0);
            }



            // Method will be called if the camera cannot be opened
            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Init Error", errorCode);
            }



        });

        dashboard.updateConfig();
        testPacket.put("test", R1X);
        testPacket.put("Status", "Ready to Start!");
        testPacket.put("Elapsed Time", "-");
        dashboard.sendTelemetryPacket(testPacket);

        waitForStart();

        timer.reset();

        driveThread.start();
        //intakeThread.start();

        while(opModeIsActive()){

           // drive.

            testPacket.put("Status", "Opmode Active");
            testPacket.put("Elapsed Time", timer.seconds());
            testPacket.fieldOverlay().setFill("blue").fillRect(-20, -20, 40, 40);

            dashboard.sendTelemetryPacket(testPacket);





        }


    }

    private void driveUpdate() {
        if (gamepad1.right_bumper) { // replace this with a button for sprint
            drive.setPower(new Vector2D(gamepad1.left_stick_x * SPRINT_LINEAR_MODIFIER, gamepad1.left_stick_y * SPRINT_LINEAR_MODIFIER), gamepad1.right_stick_x * SPRINT_ROTATIONAL_MODIFIER, false);
        } else {
            drive.setPower(new Vector2D(gamepad1.left_stick_x * NORMAL_LINEAR_MODIFIER, gamepad1.left_stick_y * NORMAL_LINEAR_MODIFIER), gamepad1.right_stick_x * NORMAL_ROTATIONAL_MODIFIER, false);
        }
    }



    public void changePower(){
        if(power == 0.3){
            power = 0.5;
        }else if(power == 0.5){
            power = 0.7;
        }else if(power == 0.7){
            power = 1;
        }else{
            power = 0.3;
        }
    }
}