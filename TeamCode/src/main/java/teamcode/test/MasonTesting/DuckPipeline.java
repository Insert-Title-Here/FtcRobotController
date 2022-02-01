//package teamcode.test.MasonTesting;
//
//import org.opencv.core.Core;
//import org.opencv.core.Mat;
//import org.opencv.core.MatOfPoint;
//import org.opencv.core.Point;
//import org.opencv.core.Scalar;
//import org.opencv.core.Size;
//import org.opencv.imgproc.Imgproc;
//import org.opencv.utils.Converters;
//import org.openftc.easyopencv.OpenCvPipeline;
//
//import java.util.ArrayList;
//
//import teamcode.common.MecanumDriveTrain;
//
//public class DuckPipeline extends OpenCvPipeline {
//    Mat frameHSV, frameBlurred, frameMasked, frameBinary;
//    ArrayList<MatOfPoint> contours, subContours;
//    ArrayList<Integer> xList, yList;
//    double duckX;
//    int xMin, xMax;
//
//    private final Scalar lowerYellow = new Scalar(13, 130, 100);
//    private final Scalar upperYellow = new Scalar(29, 255, 255);
//
//    private final int deviation = 5;
//
//    @Override
//    public Mat processFrame(Mat frame) {
//        Mat frameHSV = new Mat();
//        Mat frameBlurred = new Mat();
//        Mat frameMasked = new Mat();
//        Mat frameBinary = new Mat();
//        contours = new ArrayList<>();
//        subContours = new ArrayList<>();
//        xMin = Integer.MAX_VALUE;
//        xMax = Integer.MIN_VALUE;
//
//        /*
//        1. Blur the image
//        2. Change the color space
//        3. Get the mask of the image
//        4. Add erosion and dilation for noise reduction
//        5. Get the binary of the image
//        6. Get the contours and create subdivided contour lists based on arc length
//        7. Get x and y cords in frame
//        8. Determine whether the robot should strafe -x or x amt
//        9. Get the duck
//         */
//
//        // 1
//        Imgproc.GaussianBlur(frame, frameBlurred, new Size(7, 7), 0);
//
//        // 2 + 3
//        Imgproc.cvtColor(frameBlurred, frameHSV, Imgproc.COLOR_RGB2HSV);
//        Core.inRange(frameHSV, lowerYellow, upperYellow, frameMasked);
//
//        // 4
//        Imgproc.erode(frameMasked, frameMasked, new Mat(), new Point(-1, -1), 2);
//        Imgproc.dilate(frameMasked, frameMasked, new Mat(), new Point(-1, -1), 2);
//
//        // 5
//        Imgproc.threshold(frameMasked, frameBinary, 100, 255, Imgproc.THRESH_BINARY_INV);
//
//        // 6 (needs are length implementation)
//        Imgproc.findContours(frameMasked, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
//
//        // 7
//        if (contours == null) {
//            return null;
//        }
//
//
//        double centerMin, centerMax;
//        centerMin = frame.width() / 2 - deviation;
//        centerMax = frame.width() / 2 + deviation;
//
//        // strafe thing?
//
//
//        do {
//            xCord();
//            // strafe either left or right using modulus based on what duck x is
//            // like in relation to centerMin and centerMax
//        } while (duckX < centerMin || duckX > centerMax);
//
//        // go forward to get duck
//
//
//        return null;
//    }
//
////    public void xCord(ArrayList<MatOfPoint> ) {
////        ArrayList<Point> pointList = new ArrayList<>();
////        Converters.Mat_to_vector_Point(contours.get(0), pointList);
////        xList = new ArrayList<>();
////        yList = new ArrayList<>();
////
////        xList.add((int) pointList.get(0).x);
////
////
////        for (int val : xList) {
////            if (val < xMin) {
////                xMin = val;
////            }
////
////            if (val > xMax) {
////                xMax = val;
////            }
////        }
////
////        duckX = xMin + Math.abs((xMax - xMin) / 2);
////    }
//}
