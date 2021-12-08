package teamcode.test.MasonTesting;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import java.util.ArrayList;
import java.util.Scanner;
import java.util.Set;
import java.util.TreeMap;
import java.util.TreeSet;

import org.checkerframework.checker.units.qual.A;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.utils.Converters;
import org.opencv.videoio.VideoCapture;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class CvDetectionPipeline extends OpenCvPipeline {

    ArrayList<Double> xPList = new ArrayList<>();
    ArrayList<Double> yPList = new ArrayList<>();

    final Scalar lower_yellow = new Scalar(0, 125, 125);
    final Scalar upper_yellow = new Scalar(50, 255, 255);

    final int focalLength = 32;
    final double cameraAngle = 68;

    @Override
    public synchronized Mat processFrame(Mat frame){
        // mat vars
        Mat hsv = new Mat();
        Mat blurred = new Mat();
        Mat mask = new Mat();
        Mat binary = new Mat();
        ArrayList<MatOfPoint> contours = new ArrayList<>();

        // change src mat to be frame param
        Mat src = new Mat();
        frame.copyTo(src);

        // apply blur, change color, get range
        Imgproc.GaussianBlur(src, blurred, new Size(7, 7), 0);
        Imgproc.cvtColor(blurred, hsv, Imgproc.COLOR_BGR2HSV);
        Core.inRange(hsv, lower_yellow, upper_yellow, mask);

        // erosion and dilation noise reduction
        Imgproc.erode(mask, mask, new Mat(), new Point(-1, -1), 2);
        Imgproc.dilate(mask, mask, new Mat(), new Point(-1, -1), 2);

        // get binary threshold
        Imgproc.threshold(mask, binary, 100, 255, Imgproc.THRESH_BINARY_INV);

        // get contours and create subdivided contours list
        Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        ArrayList<MatOfPoint> subContours = new ArrayList<>();

        // get arc length to get the needed individual cubes
//        for (int i = 0; i < contours.size(); i++) {
//            double val = Imgproc.arcLength(new MatOfPoint2f(contours.get(i).toArray()), true);
//            if (val > 25) {
//                subContours.add(contours.get(i));
//            }
//        }

        // get x and y vals + x and y cords
        xPList.clear();
        yPList.clear();
        for (int i = 0; i < contours.size(); i++) {
            ArrayList<Point> pointList = new ArrayList<>();
            Converters.Mat_to_vector_Point(contours.get(i), pointList);
            ArrayList<Integer> xList = new ArrayList<>();
            ArrayList<Integer> yList = new ArrayList<>();

            for (Point p : pointList) {
                xList.add((int) p.x);
                yList.add((int) p.y);
            }

            int xMin = Integer.MAX_VALUE;
            int xMax = Integer.MIN_VALUE;
            for (int num : xList) {
                if (num < xMin) {
                    xMin = num;
                }

                if (num > xMax) {
                    xMax = num;
                }
            }

            int yMin = Integer.MAX_VALUE;
            int yMax = Integer.MIN_VALUE;
            for (int num : yList) {
                if (num < yMin) {
                    yMin = num;
                }

                if (num > yMax) {
                    yMax = num;
                }
            }

            int xCord = xMin + Math.abs((xMax - xMin) / 2);
            int yCord = yMin + Math.abs((yMax - yMin) / 2);
            int objWidth = Math.abs(xMax - xMin);
            int objHeight = Math.abs(yMax - xMin);
            Imgproc.drawMarker(src, new Point(xCord, yCord), new Scalar(255, 255, 255));

            int srcWidth = src.width();
            int srcHeight = src.height();

            // get distance
            double distance = distanceFinder(50.8, objWidth);

            // flip around x/y coordinates due to how the robot uses vector movement
            double yPos = xCord; double xAmt = 0;
            xAmt = distance * Math.sin(cameraAngle);
            xAmt = Math.round((xAmt * 100.0) / 100.0);
            xPList.add(xAmt);
            yPList.add(yPos);
        }
        Imgproc.line(src, new Point(src.width() / 2, src.height()), new Point(src.width() / 2, 0), new Scalar(255, 255, 255));
        Imgproc.drawContours(src, contours, -1, new Scalar(255, 255, 255), 2, Imgproc.LINE_8);
        return src;
    }

    public double distanceFinder(double realWidth, double pixelWidth) {
        return (realWidth * focalLength) / pixelWidth;
    }

    public ArrayList<Double> xPointList() {
        return xPList;
    }

    public ArrayList<Double> yPointList() {
        return yPList;
    }
}