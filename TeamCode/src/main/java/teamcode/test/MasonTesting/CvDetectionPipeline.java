package teamcode.test.MasonTesting;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import java.util.ArrayList;
import java.util.Scanner;
import java.util.Set;
import java.util.TreeMap;
import java.util.TreeSet;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.checkerframework.checker.units.qual.A;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.utils.Converters;
import org.opencv.videoio.VideoCapture;
import org.openftc.easyopencv.OpenCvPipeline;

import teamcode.common.Debug;

public class CvDetectionPipeline extends OpenCvPipeline {

    Scalar col = new Scalar(0, 0, 0);

    ArrayList<Double> xPList = new ArrayList<>();
    ArrayList<Double> yPList = new ArrayList<>();

    final Scalar cali_lower_yellow_3 = new Scalar(13, 130, 100);
    final Scalar cali_upper_yellow_3 = new Scalar(29, 255, 255);

    // 32, 0.47, 12
    double focalLength = 12;
    // 50.8mm
    double objectHeight = 50.8;
    // mm
    double sensorHeight = 24;
    final double cameraAngle = 68;

    // distance in inches
    double distance = 0.0;

    @Override
    public void init(Mat frame) {
        xPList.add(0.0);
        yPList.add(0.0);
    }

    @Override
    public synchronized Mat processFrame(Mat frame){
        // mat vars
        Mat hsv = new Mat();
        Mat blurred = new Mat();
        Mat mask = new Mat();
        Mat binary = new Mat();
        ArrayList<MatOfPoint> contours = new ArrayList<>();

        // change src mat to be frame param

        // apply blur, change color, get range
        Imgproc.GaussianBlur(frame, blurred, new Size(7, 7), 0);
        Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);
        Core.inRange(hsv, cali_lower_yellow_3, cali_upper_yellow_3, mask);

        // erosion and dilation noise reduction
        Imgproc.erode(mask, mask, new Mat(), new Point(-1, -1), 2);
        Imgproc.dilate(mask, mask, new Mat(), new Point(-1, -1), 2);

        // get binary threshold
        Imgproc.threshold(mask, binary, 100, 255, Imgproc.THRESH_BINARY_INV);

        // get contours and create subdivided contours list
        Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        ArrayList<MatOfPoint> subContours = new ArrayList<>();

        // get arc length to get the needed individual cubes
        for (int i = 0; i < contours.size(); i++) {
            double val = Imgproc.arcLength(new MatOfPoint2f(contours.get(i).toArray()), true);
            if (val > 75 && val < 500) {
                subContours.add(contours.get(i));
            }
        }

        // get x and y vals + x and y cords
        xPList.clear();
        yPList.clear();
        for (int i = 0; i < subContours.size(); i++) {
            ArrayList<Point> pointList = new ArrayList<>();
            Converters.Mat_to_vector_Point(subContours.get(i), pointList);
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
            Imgproc.drawMarker(frame, new Point(xCord, yCord), new Scalar(255, 255, 255));

            int srcWidth = frame.width();
            int srcHeight = frame.height();

            distance = distance(srcHeight, objHeight);

            // get distance
//            double distance = distanceFinder(2.0, objWidth);
////
////            // flip around x/y coordinates due to how the robot uses vector movement
//            double xAmt = distance * Math.sin(cameraAngle);
//            xPList.add(Math.abs(xAmt));
////            double yPos = xCord; double xAmt = 0;
////            xAmt = distance * Math.sin(cameraAngle);
////            xAmt = Math.round((xAmt * 100.0) / 100.0);
////            xPList.add(xAmt);
////            yPList.add(yPos);
//            yPList.add(0.0);
        }
        Debug.log("DIST: " + distance);
        Imgproc.line(frame, new Point(frame.width() / 2, frame.height()), new Point(frame.width() / 2, 0), new Scalar(255, 255, 255));
        Imgproc.drawContours(frame, subContours, -1, new Scalar(255, 255, 255), 2, Imgproc.LINE_8);
        // comment part means src to frame, change back to src when comment heap is done
        // memory fix?
        hsv.release();
        blurred.release();
        mask.release();
        binary.release();

        return frame;
    }

    // gets distance to object in mm
    public double distance(int imageHeight, double objectHeight) {
        // divide by 25.4 to get inches from mm
        return ((focalLength * objectHeight * imageHeight) / (objectHeight * sensorHeight)) / 25.4;
    }

    public double distanceToObj() {
        return distance;
    }

    public synchronized ArrayList<Double> xPointList() {
        return xPList;
    }

    public synchronized ArrayList<Double> yPointList() {
        return yPList;
    }

    public Scalar color() {
        return col;
    }
}