package teamcode.test.MasonTesting;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import java.util.ArrayList;
import java.util.HashMap;
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

public class DistanceDetection extends OpenCvPipeline {

    ArrayList<Double> xPList = new ArrayList<>();
    ArrayList<Double> yPList = new ArrayList<>();
    ArrayList<Freight> freightList = new ArrayList<>();
    private double lowestY = 0.0;
    private double correlX = 0.0;

    final Scalar cali_lower_yellow_3 = new Scalar(13, 130, 100);
    final Scalar cali_upper_yellow_3 = new Scalar(29, 255, 255);

    class Freight {
        private final int focalLength = 12;      // mm
        private final double ballWidth = 69.85;  // 2.75 in
        private final double ballHeight = 69.85; // 2.75 in
        private final double cubeWidth = 50.8;   // 2 in
        private final double cubeHeight = 50.8;  // 2 in

        private final int imageHeight = 240;     // px
        private final int imageWidth = 320;      // px

        private final double sensorWidth = 31.9; // mm
        private final double sensorHeight = 24;  // mm

        public int posX;
        public int posY;
        public int objWidth;
        public int objHeight;
        public boolean isCube;
        private double distance;

        public Freight(int posX, int posY, int objWidth, int objHeight, boolean isCube) {
            this.posX = posX;
            this.posY = posY;
            this.objWidth = objWidth;
            this.objHeight = objHeight;
            this.isCube = isCube;
            this.distance = -1;
        }

        // sets the distance in millimeters
        /*
        public double getDistance() {
            if (this.isCube) {
                this.distance = (((focalLength * cubeHeight * imageHeight) / (this.objHeight * this.sensorHeight))
                        + ((focalLength * cubeWidth * imageWidth) / (this.objWidth * sensorWidth))) / 2;
            } else {
                this.distance = (((focalLength * ballHeight * imageHeight) / (this.objHeight * this.sensorHeight))
                        + ((focalLength * ballWidth * imageWidth) / (this.objWidth * sensorWidth))) / 2;
            }
            return this.distance;
        } */
    }

    @Override
    public void init(Mat mat) {

    }

    @Override
    public void onViewportTapped() {

    }

    @Override
    public synchronized Mat processFrame(Mat frame) {
        Mat hsv = new Mat();
        Mat blurred = new Mat();
        Mat mask = new Mat();
        Mat binary = new Mat();
        ArrayList<MatOfPoint> contours = new ArrayList<>();

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

        for (int i = 0; i < contours.size(); i++) {
            double val = Imgproc.arcLength(new MatOfPoint2f(contours.get(i).toArray()), true);
            if (val > 75 && val < 500) {
                subContours.add(contours.get(i));
            }
        }

        lowestY = 0.0;
        correlX = 0.0;
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
                xMin = (num < xMin) ? num : xMin;
                xMax = (num > xMax) ? num : xMax;
            }

            int yMin = Integer.MAX_VALUE;
            int yMax = Integer.MIN_VALUE;
            for (int num : yList) {
                yMin = (num < yMin) ? num : yMin;
                yMax = (num > yMax) ? num : yMax;
            }

            xList.clear();
            yList.clear();

            int centerX = xMin + Math.abs((xMax - xMin) / 2);
            int centerY = yMin + Math.abs((yMax - yMin) / 2);
            Debug.log(centerY);
            Imgproc.line(frame, new Point(160, 0), new Point(160, 240), new Scalar(255, 255, 255));
            double theta = Math.atan((double)(240 - centerY) / (double)(centerX - 160));
            Imgproc.putText(frame, String.valueOf(theta), new Point(xMax, centerY), Imgproc.FONT_HERSHEY_COMPLEX, 1, new Scalar(0, 255, 255), 2, Imgproc.LINE_AA);
            //int objW = Math.abs(xMax - xMin);
            //int objH = Math.abs(yMax - yMin);
            //freightList.add(new Freight(centerX, centerY, objW, objH, true));
        }

        Imgproc.drawContours(frame, subContours, -1, new Scalar(255, 255, 255), 2, Imgproc.LINE_8);
        Imgproc.putText(frame, lowestY + ", " + correlX, new Point(10, 230), Imgproc.FONT_HERSHEY_COMPLEX, 1, new Scalar(0, 255, 0), 2, Imgproc.LINE_AA);

        hsv.release();
        blurred.release();
        mask.release();
        binary.release();

        return frame;
    }

    public Mat distanceFinder(Mat frame) {
        Mat hsv = new Mat();
        Mat blurred = new Mat();
        Mat mask = new Mat();
        Mat binary = new Mat();
        ArrayList<MatOfPoint> contours = new ArrayList<>();

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

        for (int i = 0; i < contours.size(); i++) {
            double val = Imgproc.arcLength(new MatOfPoint2f(contours.get(i).toArray()), true);
            if (val > 75 && val < 500) {
                subContours.add(contours.get(i));
            }
        }

        lowestY = Double.MAX_VALUE;
        correlX = 0.0;
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
                xMin = (num < xMin) ? num : xMin;
                xMax = (num > xMax) ? num : xMax;
            }

            int yMin = Integer.MAX_VALUE;
            int yMax = Integer.MIN_VALUE;
            for (int num : yList) {
                yMin = (num < yMin) ? num : yMin;
                yMax = (num > yMax) ? num : yMax;
            }

            xList.clear();
            yList.clear();

            int centerX = xMin + Math.abs((xMax - xMin) / 2);
            int centerY = yMin + Math.abs((yMax - yMin) / 2);
            if (centerY < lowestY) {
                lowestY = centerY;
                correlX = centerX;
            }
            //int objW = Math.abs(xMax - xMin);
            //int objH = Math.abs(yMax - yMin);
            //freightList.add(new Freight(centerX, centerY, objW, objH, true));
        }

        Imgproc.drawContours(frame, subContours, -1, new Scalar(255, 255, 255), 2, Imgproc.LINE_8);

        hsv.release();
        blurred.release();
        mask.release();
        binary.release();

        return frame;
    }

    public String getNearestDistance() {
        return "LowestY, X: " + lowestY + ", " + correlX;
    }
}
