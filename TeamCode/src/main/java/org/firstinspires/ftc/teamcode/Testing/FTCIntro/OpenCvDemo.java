//package org.firstinspires.ftc.teamcode.Testing.FTCIntro;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.opencv.core.Core;
//import org.opencv.core.Mat;
//import org.opencv.core.Point;
//import org.opencv.core.Rect;
//import org.opencv.core.Scalar;
//import org.opencv.imgproc.Imgproc;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvPipeline;
//import org.openftc.easyopencv.OpenCvWebcam;
//
//public class OpenCvDemo extends LinearOpMode {
//
//
//    private OpenCvWebcam webcam;
//    WebcamName wc;
//    private ExamplePipeline pipeline;
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        WebcamName wc = hardwareMap.get(WebcamName.class, "Webcam");
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(wc, cameraMonitorViewId);
//        pipeline = new ExamplePipeline();
//        webcam.setPipeline(pipeline);
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(320, 240);
//            }
//
//            @Override
//            public void onError(int errorCode) {
//
//            }
//        });
//    }
//
//    private class ExamplePipeline extends OpenCvPipeline{
//
//        public enum BarcodePosition {
//            LEFT,
//            CENTER,
//            RIGHT
//        }
//
//        public enum Side {
//            RED,
//            BLUE
//        }
//
//        final Scalar BLUE = new Scalar(0, 0, 255);
//        final Scalar GREEN = new Scalar(0, 255, 0);
//
//         Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(123, 150);
//         Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(270, 150);
//
//        static final int REGION_WIDTH = 40;
//        static final int REGION_HEIGHT = 80;
//
//        Point region1_pointA = new Point(
//                REGION1_TOPLEFT_ANCHOR_POINT.x,
//                REGION1_TOPLEFT_ANCHOR_POINT.y);
//        Point region1_pointB = new Point(
//                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
//                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
//        Point region2_pointA = new Point(
//                REGION2_TOPLEFT_ANCHOR_POINT.x,
//                REGION2_TOPLEFT_ANCHOR_POINT.y);
//        Point region2_pointB = new Point(
//                REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
//                REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
//
//        Mat region1 = new Mat();
//        Mat region2 = new Mat();
//        Mat BGR = new Mat();
//        Mat bMat = new Mat();
//        int avg1 = 0, avg2 = 0;
//
//        volatile BarcodePosition position = BarcodePosition.RIGHT;
//        volatile Side side = Side.RED;
//
//        void inputToB(Mat input) {
//            Imgproc.cvtColor(input, BGR, Imgproc.COLOR_RGB2HLS);
//            Core.extractChannel(BGR, bMat, 1);
//        }
//
//        @Override
//        public void init(Mat input) {
//        }
//
//        @Override
//        public Mat processFrame(Mat input) {
//            inputToB(input);
//
//            if (side == Side.RED) {
//                REGION1_TOPLEFT_ANCHOR_POINT = new Point(108, 85);
//                REGION2_TOPLEFT_ANCHOR_POINT = new Point(270, 85);
//            } else {
//                REGION1_TOPLEFT_ANCHOR_POINT = new Point(0, 85);
//                REGION2_TOPLEFT_ANCHOR_POINT = new Point(151, 85);
//            }
//            Point region1_pointA = new Point(
//                    REGION1_TOPLEFT_ANCHOR_POINT.x,
//                    REGION1_TOPLEFT_ANCHOR_POINT.y);
//            Point region1_pointB = new Point(
//                    REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
//                    REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
//            Point region2_pointA = new Point(
//                    REGION2_TOPLEFT_ANCHOR_POINT.x,
//                    REGION2_TOPLEFT_ANCHOR_POINT.y);
//            Point region2_pointB = new Point(
//                    REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
//                    REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
//
//            region1 = bMat.submat(new Rect(region1_pointA, region1_pointB));
//            region2 = bMat.submat(new Rect(region2_pointA, region2_pointB));
//
//            avg1 = (int) Core.mean(region1).val[0];
//            avg2 = (int) Core.mean(region2).val[0];
//            int min = Math.min(avg1, avg2);
//
//            Imgproc.rectangle(
//                    input,
//                    region1_pointA,
//                    region1_pointB,
//                    BLUE,
//                    2
//            );
//            Imgproc.rectangle(
//                    input,
//                    region2_pointA,
//                    region2_pointB,
//                    BLUE,
//                    2
//            );
//
//            if (side == MecanumBarcodePipeline.Side.RED) {
//                if (Math.abs(avg1 - avg2) < 20) {
//                    position = BarcodePosition.RIGHT;
//                } else if (min == avg2) {
//                    position = BarcodePosition.CENTER;
//                } else {
//                    position = BarcodePosition.LEFT;
//                }
//            } else {
//                if (Math.abs(avg1 - avg2) < 20) {
//                    position = BarcodePosition.LEFT;
//                } else if (min == avg1) {
//                    position = BarcodePosition.CENTER;
//                } else {
//                    position = BarcodePosition.RIGHT;
//                }
//            }
//          return input;
//        }
//
//    /*
//    input.get(1, 1, new byte[1]);
//        Imgproc.watershed(new Mat(), new Mat());
//     */
//
//        public BarcodePosition getPos() {
//            return position;
//        }
//
//        public void setSide(Side side) {
//            this.side = side;
//        }
//
//        public Side getSide() {
//            return side;
//        }
//    }
//}
