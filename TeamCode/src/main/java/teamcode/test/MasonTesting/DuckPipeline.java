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

public class DuckPipeline extends OpenCvPipeline {

    private int direction;
    private boolean duckCentered = false;
    private boolean duckInFrame = false;
    private ArrayList<MatOfPoint> contours, contourLengths;
    private ArrayList<Integer> xList, yList;

    private double xCenterMin, xCenterMax;
    private final double deviation = 25;

    private final Scalar lowerYellow = new Scalar(13, 130, 100);
    private final Scalar upperYellow = new Scalar(29, 255, 255);

    private final int blurSize = 7;

    @Override
    public Mat processFrame(Mat input) {
        xList = new ArrayList<>();
        yList = new ArrayList<>();

        Mat frameHSV = new Mat();
        Mat frameBlurred = new Mat();
        Mat frameMasked = new Mat();
        Mat frameBinary = new Mat();
        contours = new ArrayList<>();
        contourLengths = new ArrayList<>();

        xCenterMin = input.width() / 2 - deviation;
        xCenterMax = input.width() / 2 + deviation;

        // blur the image
        Imgproc.GaussianBlur(input, frameBlurred, new Size(blurSize, blurSize), 0);

        // color space and masking
        Imgproc.cvtColor(frameBlurred, frameHSV, Imgproc.COLOR_RGB2HSV);
        Core.inRange(frameHSV, lowerYellow, upperYellow, frameMasked);

        // erosion and dilation for noise reduction
        Imgproc.erode(frameMasked, frameMasked, new Mat(), new Point(-1, -1), 2);
        Imgproc.dilate(frameMasked, frameMasked, new Mat(), new Point(-1, -1), 2);

        // get the binarys of the image
        Imgproc.threshold(frameMasked, frameBinary, 100, 255, Imgproc.THRESH_BINARY_INV);

        // get contours and arc length
        Imgproc.findContours(frameMasked, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        for (int i = 0; i < contours.size(); i++) {
            double indexedLength = Imgproc.arcLength(new MatOfPoint2f(contours.get(i).toArray()), true);
            if (indexedLength >= 30 && indexedLength <= 220) {
                contourLengths.add(contours.get(i));
            }
        }

        xList.clear();
        yList.clear();

        for (int i = 0; i < contourLengths.size(); i++) {
            ArrayList<Point> pointList = new ArrayList<>();
            Converters.Mat_to_vector_Point(contourLengths.get(i), pointList);

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

            double xCord = xMin + Math.abs((xMax - xMin) / 2);


            if (xCord < xCenterMin) {
                setDirection(-1);
            } else if (xCord > xCenterMax) {
                setDirection(1);
            } else {
                setCentered(true);
                setDirection(0);
            }
        }
        Imgproc.drawContours(input, contourLengths, -1, new Scalar(255, 255, 255), 1, Imgproc.LINE_8);

        frameHSV.release();
        frameBlurred.release();
        frameMasked.release();
        frameBinary.release();
        contours.clear();
        contourLengths.clear();
        xList.clear();
        yList.clear();
        return input;
    }

    public boolean isCentered() {
        return duckCentered;
    }

    public double direction() {
        return direction;
    }

    private void setDirection(int dir) {
        direction = dir;
    }

    private void setCentered(boolean state) {
        duckCentered = state;

    }
}
