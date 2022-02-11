//// TEST SCRIPT FOR OPENCV BY MASON
//// link for reference: https://github.com/OpenFTC/EasyOpenCV/blob/master/doc/user_docs/camera_initialization_overview.md
//
//package teamcode.test.MasonTesting;
//
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
//
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//
//import org.opencv.core.Scalar;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//
//import java.util.ArrayList;
//
//import teamcode.common.AbstractOpMode;
//
//@TeleOp(name = "RT-Object Detection")
//public class CvRTT extends AbstractOpMode {
//
//    // Get webcam and create an OpenCvCamera
//    WebcamName wc;
//    OpenCvCamera camera;
//
//    Thread focalThread;
//
//    // global obj
//    static final ColorCalibration colCalibrator = new ColorCalibration();
//
//    @Override
//    protected void onInitialize() {
//
//        // Init webcam and create a cam object using CvFactory
//        // Make sure to have the name of the webcam set in the config settings of the robot
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        wc = hardwareMap.get(WebcamName.class, "Webcam");
//
//        // W/ or W/ out live preview
//        camera = OpenCvCameraFactory.getInstance().createWebcam(wc, cameraMonitorViewId);
//        // camera = OpenCvCameraFactory.getInstance().createWebcam(wc);
//
//        camera.setPipeline(dp);
//
//        // Open an asynchronous connection to the device
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//
//            // Start opening the camera and stream it
//            @Override
//            public void onOpened() {
//
//                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            // Method will be called if the camera cannot be opened
//            @Override
//            public void onError(int errorCode) {
//                telemetry.addData("Camera Init Error", errorCode);
//            }
//        });
//
//        focalThread = new Thread() {
//            public void run() {
//                while (opModeIsActive()) {
//                    focalController();
//                }
//            }
//        };
//        previousDpadUpState = false;
//        previousDpadDownState = false;
//    }
//
//    @Override
//    protected void onStart() {
//        // Keep the op mode running, to keep the system from coming to a halt
//        focalThread.start();
////            try {
////                Thread.sleep(50);
////                if (dp.xPointList() != null) {
////                    for (int i = 0; i < dp.xPointList().size(); i++) {
////                        telemetry.addData("X: " + i + " ", dp.xPointList().get(i));
////                    }
////
////                }
////                telemetry.addData("Focal Length: ", dp.focalLength);
////                telemetry.update();
//////                telemetry.addData("R", colCalibrator.hValue());
//////                telemetry.addData("G", colCalibrator.sValue());
//////                telemetry.addData("B", colCalibrator.vValue());
//////                telemetry.update();
////            } catch (InterruptedException e) {
////                e.printStackTrace();
////            }
//        while (opModeIsActive()) {
//            if (dp.xPointList() != null || dp.xPointList().size() != 0) {
//                    telemetry.addData("Forward-", dp.xPointList() .toString());
//            }
//            telemetry.addData("FocalLength: ", dp.focalLength);
//            telemetry.update();
//        }
//
//    }
//
//    @Override
//    protected void onStop() {
//        camera.stopStreaming();
//        focalThread.interrupt();
//    }
//
//    boolean previousDpadUpState, previousDpadDownState;
//    public void focalController() {
//        if (gamepad1.dpad_up && !previousDpadUpState) {
//            dp.focalLength += 0.5;
//        } else if (gamepad1.dpad_down && !previousDpadDownState) {
//            dp.focalLength -= 0.5;
//        }
//        previousDpadDownState = gamepad1.dpad_down;
//        previousDpadUpState = gamepad1.dpad_up;
//
//    }
//}
