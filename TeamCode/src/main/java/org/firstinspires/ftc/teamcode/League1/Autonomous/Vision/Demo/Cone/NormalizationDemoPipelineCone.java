package org.firstinspires.ftc.teamcode.League1.Autonomous.Vision.Demo.Cone;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.League1.Subsystems.MecDrive;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Config
public class NormalizationDemoPipelineCone extends OpenCvPipeline {

    public boolean isNormalizing;
    public int normalizeTarget;

    // Configuration variables for isolating cone color
    public static int YLower = 0;
    public static int CrLower = 90;
    public static int CbLower = 160;
    public static int YUpper = 200;
    public static int CrUpper = 150;
    public static int CbUpper = 255;

    // Make it so that the mat returned can be changed in dashboard
    public static int returnMat = 0;

    // Define mats
    Mat temp = new Mat();
    Mat original = new Mat();

    // Define telemetry variable
    Telemetry telemetry;

    // Define lists
    ArrayList<Integer> xList, yList;
    ArrayList<Double> contourAreas;

    // Define ints
    int cX, cY;
    double maxArea = 0;
    int maxAreaIndex = 0;
    int longestContourX = 0;
    int longestContourY = 0;

    // Don't really know what this thing is, but we're defining it
    Moments M;

    // Our DriveTrain class
    MecDrive drive;

    //Constructor
    public NormalizationDemoPipelineCone(Telemetry telemetry, MecDrive drive){
        // Set up lists and telemetry
        xList = new ArrayList<>();
        yList = new ArrayList<>();
        contourAreas = new ArrayList<>();
        this.telemetry = telemetry;
        this.drive = drive;
    }

    @Override
    public Mat processFrame(Mat input) {
        original = input;

        // Convert to YCrCb color space
        Imgproc.cvtColor(input, temp, Imgproc.COLOR_RGB2YCrCb);

        // Blur image to reduce noise
        Imgproc.GaussianBlur(temp, temp, new Size(5, 5), 0);

        // Make binary image of yellow pixels
        Core.inRange(temp, new Scalar(YLower, CrLower, CbLower), new Scalar(YUpper, CrUpper, CbUpper), temp);

        // Find all contours in binary image
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(temp, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        for(int i = 0; i < contours.size(); i++){
            // Filter out small, irrelevant contours
            if(Imgproc.contourArea(contours.get(i)) > 250) {

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

                // Calculate the area of the contour and add it to a list
                contourAreas.add(Imgproc.contourArea(contours.get(i)));

            }
        }

        // Find the largest contour
        maxArea = 0;
        for(int i = 0; i < xList.size() && i < contourAreas.size() && i < yList.size(); i++) {
            if(contourAreas.get(i) > maxArea) {
                maxArea = contourAreas.get(i);
                maxAreaIndex = i;
            }
        }

        // Make sure the program doesn't crash if no contours are found
        if(contourAreas.size() > 0) {
            // Find the coordinates of the largest contour and display it on the screen
            longestContourX = xList.get(maxAreaIndex);
            longestContourY = yList.get(maxAreaIndex);
            Imgproc.circle(input, new Point(xList.get(maxAreaIndex), yList.get(maxAreaIndex)), 3, new Scalar(0, 255, 0));
        }

        // Telemetry stuff
        telemetry.addData("Contour X Pos", longestContourX);
        telemetry.addData("Contour Y Pos", longestContourY);
        telemetry.update();

        // Clear lists
        contourAreas.clear();
        xList.clear();
        yList.clear();

        // Represent the target position on the screen
        if(isNormalizing) {
            Imgproc.line(input, new Point(normalizeTarget, 0), new Point(normalizeTarget, 176), new Scalar(255, 0, 0), 2);
        }

        // Return the input mat to the camera stream
        if(returnMat == 0) {
            return input;
        } else {
            return temp;
        }
    }

    // Getters for coordinates of center of largest contour (cone)
    public int getXContour() {
        return longestContourX;
    }
    public int getYContour(){
        return longestContourY;
    }

    // Example of how this can be implemented
    public void normalizeStrafe(double power, int target, int tolerance) {
        ElapsedTime time = new ElapsedTime();
        double startTime = time.seconds();
        normalizeTarget = target;
        isNormalizing = true;
        int xMax = target + tolerance;
        int xMin = target - tolerance;

        while((getXContour() > xMax || getXContour() < xMin) && time.seconds() - startTime < 5) {
            if(getXContour() > xMax) {
                drive.setPowerAuto(-power, MecDrive.MovementType.STRAFE);
            } else {
                drive.setPowerAuto(power, MecDrive.MovementType.STRAFE);
            }
        }
        drive.simpleBrake();

        isNormalizing = false;
    }

}
