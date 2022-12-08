package org.firstinspires.ftc.teamcode.League1.Autonomous.Vision.Demo;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.League1.Subsystems.MecDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Config
public class NormalizationDemoPipeline extends OpenCvPipeline {

    public boolean isNormalizing;
    public int normalizeTarget;

    // Configuration variables for isolating pole color
    public static int HLower = 23;
    public static int SLower = 0;
    public static int VLower = 155;
    public static int HUpper = 50;
    public static int SUpper = 255;
    public static int VUpper = 255;

    // Make it so that the mat returned can be changed in dashboard
    public static int returnMat = 0;

    // Define mats
    Mat temp = new Mat();
    Mat original = new Mat();

    // Define telemetry variable
    Telemetry telemetry;

    // Define lists
    ArrayList<Integer> xList, yList, contourLengths;

    // Define ints
    int cX, cY;
    int maxLength = 0;
    int maxLengthIndex = 0;
    int longestContourX = 0;
    int longestContourY = 0;

    // Don't really know what this thing is, but we're defining it
    Moments M;

    // Servo
    Servo cameraServo;

    //Constructor
    public NormalizationDemoPipeline(Telemetry telemetry, Servo cameraServo){
        // Set up lists and telemetry
        xList = new ArrayList<>();
        yList = new ArrayList<>();
        contourLengths = new ArrayList<>();
        this.telemetry = telemetry;
        this.cameraServo = cameraServo;
    }

    @Override
    public Mat processFrame(Mat input) {
        original = input;

        // Convert to HSV color space
        Imgproc.cvtColor(input, temp, Imgproc.COLOR_RGB2HSV);

        // Make binary image of yellow pixels
        Core.inRange(temp, new Scalar(HLower, SLower, VLower), new Scalar(HUpper, SUpper, VUpper), temp);

        // Blur image to reduce noise
        Imgproc.GaussianBlur(temp, temp, new Size(5, 5), 0);

        // Find all contours in binary image
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(temp, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        for(int i = 0; i < contours.size(); i++){
            // Filter out small, irrelevant contours
            if(contours.get(i).toArray().length > 20) {

                // Draw all contours to the screen
                Imgproc.drawContours(input, contours, i, new Scalar(255, 0, 255), 3);

                // Find center of contour and add a point on the screen
                M = Imgproc.moments(contours.get(i));
                cX = (int)(M.m10 / M.m00);
                cY = (int)(M.m01 / M.m00);

                Imgproc.circle(input, new Point(cX, cY), 3, new Scalar(0, 0, 255));

                // Save the contour's center in a list
                xList.add(cX);
                yList.add(cY);

                // Calculate the length of the contour and add it to a list
                contourLengths.add(contours.get(i).toArray().length);

            }
        }

        // Find the largest contour
        maxLength = 0;
        for(int i = 0; i < xList.size() && i < contourLengths.size() && i < yList.size(); i++) {
            if(contourLengths.get(i) > maxLength) {
                maxLength = contourLengths.get(i);
                maxLengthIndex = i;
            }
        }

        // Make sure the program doesn't crash if no contours are found
        if(contourLengths.size() > 0) {
            // Find the coordinates of the largest contour and display it on the screen
            longestContourX = xList.get(maxLengthIndex);
            longestContourY = yList.get(maxLengthIndex);
            Imgproc.circle(input, new Point(xList.get(maxLengthIndex), yList.get(maxLengthIndex)), 3, new Scalar(0, 255, 0));
        }

        // Telemetry stuff
        telemetry.addData("Contour X Pos", longestContourX);
        telemetry.addData("Contour Y Pos", longestContourY);
        telemetry.update();

        // Clear lists
        contourLengths.clear();
        xList.clear();
        yList.clear();

        if(isNormalizing) {
            Imgproc.line(input, new Point(0, normalizeTarget), new Point(320, normalizeTarget), new Scalar(255, 0, 0), 2);
        }

        // Return the input mat to the camera stream
        if(returnMat == 0) {
            return input;
        } else {
            return temp;
        }
    }

    // Getters for coordinates of center of largest contour (pole)
    public int getXContour() {
        return longestContourX;
    }
    public int getYContour(){
        return longestContourY;
    }

    public void normalizeToPole(int target, int tolerance) {
        normalizeTarget = target;
        isNormalizing = true;
        int xMax = target + tolerance;
        int xMin = target - tolerance;

        while((getYContour() > xMax || getYContour() < xMin)) {
            if(getYContour() > xMax) {
                cameraServo.setPosition(cameraServo.getPosition() - 0.005);
            } else {
                cameraServo.setPosition(cameraServo.getPosition() + 0.005);
            }

            try {
                Thread.sleep(200);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

        }
        isNormalizing = false;
    }

}
