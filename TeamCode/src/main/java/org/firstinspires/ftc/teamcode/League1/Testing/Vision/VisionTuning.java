package org.firstinspires.ftc.teamcode.League1.Testing.Vision;

//import com.acmerobotics.dashboard.config.Config;

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
import org.opencv.utils.Converters;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

//@Config
public class VisionTuning extends OpenCvPipeline {

    public static int channelToReturn = 2;

    // Configuration variables for isolating pole color
    public static int H1 = 23;
    public static int S1 = 50;
    public static int V1 = 50;
    public static int H2 = 33;
    public static int S2 = 200;
    public static int V2 = 200;

    // Config variables for signal pipeline
    public static int YUpper = 190;
    public static int YLower = 160;
    public static int CrUpper = 200;
    public static int CrLower = 130;
    public static int CbUpper = 130;
    public static int CbLower = 170;

    // Define mats
    Mat ycrcb = new Mat();
    Mat temp = new Mat();
    Mat yellow = new Mat();
    Mat red = new Mat();
    Mat blue = new Mat();

    // Define telemetry variable
    Telemetry telemetry;
    MecDrive drive;

    // Define lists
    private ArrayList<Integer> xList, yList, contourLengths;

    // Define ints
    int cX, cY;
    int maxLength = 0;
    int maxLengthIndex = 0;
    int longestContourX = 0;
    int longestContourY = 0;


    // Don't really know what this thing is, but we're defining it
    Moments M;

    // Enums
    public enum ParkPos {
        LEFT,
        RIGHT,
        CENTER
    }

    public enum PipelineMode {
        CONTOUR,
        SIGNAL
    }

    // The rectangle/submat used to evaluate the signal color
    static final Rect MIDDLE = new Rect(
            new Point(185, 15),
            new Point(220, 75)
    );

    // Sets default values for pipelineMode and position
    //PipelineMode pipelineMode = PipelineMode.SIGNAL;
    volatile ParkPos position = ParkPos.CENTER;

    public VisionTuning(Telemetry telemetry){
        // Set up lists and telemetry
        xList = new ArrayList<>();
        yList = new ArrayList<>();
        contourLengths = new ArrayList<>();
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        // Check pipelineMode and run corresponding image processing
        if(channelToReturn < 3) {

            // Convert image to YCrCb color space and extract the Y channel
            Imgproc.cvtColor(input, ycrcb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(ycrcb, yellow, 0);

            // Make a binary image of values within the desired range and calculate avg color
            Core.inRange(yellow, new Scalar(160), new Scalar(190), yellow);
            double countY = Core.mean(yellow.submat(MIDDLE)).val[0];

            // Extract Cr channel
            Core.extractChannel(ycrcb, red, 1);

            // Make binary image and calculate avg color
            Core.inRange(red, new Scalar(130), new Scalar(200), red);
            double countCr = Core.mean(red.submat(MIDDLE)).val[0];

            // Extract Cb channel
            Core.extractChannel(ycrcb, blue, 2);

            // Make binary image and calculate avg color
            Core.inRange(blue, new Scalar(130), new Scalar(170), blue);
            double countCb = Core.mean(blue.submat(MIDDLE)).val[0];

            // Telemetry
            telemetry.addData("countY", countY);
            telemetry.addData("countCr", countCr);
            telemetry.addData("countCb", countCb);

            // Check if certain channels are within certain ranges to determine color
            if(countY > 100 && countCb < 100) {
                telemetry.addData("Color", "Yellow");
                position = ParkPos.LEFT;
            } else if(countCr > 200 && countCb > 200) {
                telemetry.addData("Color", "Magenta");
                position = ParkPos.RIGHT;
            } else {
                telemetry.addData("Color", "Cyan");
                position = ParkPos.CENTER;
            }

            telemetry.update();

            // Display rectangle in detected color
            if(position == ParkPos.LEFT){
                Imgproc.rectangle(
                        input,
                        MIDDLE,
                        new Scalar(255, 255, 0),
                        3

                );

            }else if(position == ParkPos.RIGHT){
                Imgproc.rectangle(
                        input,
                        MIDDLE,
                        new Scalar(255, 0, 255),
                        3

                );

            }else{
                Imgproc.rectangle(
                        input,
                        MIDDLE,
                        new Scalar(0, 155, 255),
                        3

                );
            }

        } else {

            // Convert to HSV color space
            //Imgproc.cvtColor(input, temp, Imgproc.COLOR_RGB2YCrCb);

            // Make binary image of yellow pixels
            Core.inRange(input, new Scalar(H1, S1, V1), new Scalar(H2, S2, V2), temp);

            // Blur image to reduce noise
            Imgproc.GaussianBlur(temp, temp, new Size(5, 5), 0);

            // Find all contours in binary image
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(temp, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);


            for(int i = 0; i < contours.size(); i++){
                // Filter out small, irrelevant contours
                if(contours.get(i).toArray().length > 6) {

                    // Draw all contours to the screen
                    Imgproc.drawContours(input, contours, i, new Scalar(230, 191, 254));

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

            // Reset maxLength so the program doesn't crash - Krish is a genious
            maxLength = 0;

            // Find largest contour
            for(int i = 0; i < xList.size() && i < contourLengths.size() && i < yList.size(); i++) {
                if(contourLengths.get(i) > maxLength) {
                    maxLength = contourLengths.get(i);
                    maxLengthIndex = i;
                }
            }

            // Make sure the program doesn't crash if no contours are found
            if(contourLengths.size() > 0) {
                // Find x coordinate of largest contour and display it on the screen
                longestContourX = xList.get(maxLengthIndex);
                longestContourY = yList.get(maxLengthIndex);
                Imgproc.circle(input, new Point(xList.get(maxLengthIndex), yList.get(maxLengthIndex)), 3, new Scalar(0, 255, 0));
            }

            // Telemetry stuff
            telemetry.addData("Contour X Pos", longestContourX);
            telemetry.update();

            // Clear lists
            contourLengths.clear();
            xList.clear();
            yList.clear();

        }

        if(channelToReturn == 0) {
            return yellow;
        } else if(channelToReturn == 1) {
            return red;
        } else if(channelToReturn == 2) {
            return blue;
        } else if(channelToReturn == 3) {
            return temp;
        } else {
            return input;
        }
    }

    // Get x coordinate of center of largest contour (pole)
    public int getPolePosition() {
        return longestContourX;
    }

    public int getPoleYPos(){
        return longestContourY;
    }

    // Get parking position determined by signal mode
    public ParkPos getPosition() {
        return position;
    }

}
