package org.firstinspires.ftc.teamcode.State.Auto;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
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

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.List;

@Config
public class ConeNormalization extends OpenCvPipeline {

    // defining vars
    private Mat contourMat = new Mat(), generalMat = new Mat(), highestPole = new Mat(), hierarchy = new Mat();
    Moments M;
    private int cX;
    private  int cY;
    private List<MatOfPoint> contours = new ArrayList<>();
    private Point[] mainContour;
    private ArrayList<Double> leftContour, rightContour;
    private Point[] loopSpecificPoints;
    private double knownWidth, focalLength, perWidth, distance, leftOfContour, rightOfContour, difference, boundArea = 0;
    // for points on contour
    double numCurrent, num1Prev, num2Prev, num3Prev, num4Prev, num5Prev = 0;
    boolean toggle, toggle2, isBlue;

    // creates/accesses file
    File loggingFile = AppUtil.getInstance().getSettingsFile("telemetry.txt");
    // holds data
    public String loggingString;

    Telemetry telemetry;

    /* make stuff public static  */

    //BLUE
    public static int bluelower1 = 30; //50   50
    public static int bluelower2 = 90; //50   50
    public static int bluelower3 = 150; //10   40
    public static int blueupper1 = 130; //255   180
    public static int blueupper2 = 120; //255   200
    public static int blueupper3 = 255; //105   108//value was 80 when servo was perpendicular to the ground


    //RED
    public static int redlower1 = 0; //50   50
    public static int redlower2 = 160; //50   50
    public static int redlower3 = 50; //10   40
    public static int redupper1 = 230; //255   180
    public static int redupper2 = 235; //255   200
    public static int redupper3 = 150;
    public static double widthRemoveConstant = 3.1;


    public ConeNormalization(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        // var inits / resets
        knownWidth = 1.04; //inches
        contours = new ArrayList<>();
        // 43, 157, 166
        perWidth = 0;
        rightOfContour = 0;
        leftOfContour = 0;
        difference = 0;
        toggle = true;
        toggle2 = false;
        leftContour = new ArrayList<>();
        rightContour = new ArrayList<>();
        isBlue = true;

        generalMat = input.clone();
        //input.copyTo(generalMat);

        if(input.empty()) {
            return input;
        }
        // image tuning
        Imgproc.GaussianBlur(generalMat, contourMat, new Size(5, 5), 0);
        Imgproc.erode(contourMat, contourMat, new Mat(), new Point(-1, -1), 2);
        Imgproc.dilate(contourMat, contourMat, new Mat(), new Point(-1, -1), 2);
        Imgproc.cvtColor(contourMat, contourMat, Imgproc.COLOR_RGB2YCrCb);
//        Core.extractChannel(contourMat, contourMat, 0);
        if (isBlue) {
            Core.inRange(contourMat, new Scalar(bluelower1, bluelower2, bluelower3), new Scalar(blueupper1, blueupper2, blueupper3), contourMat);
        } else {
            Core.inRange(contourMat, new Scalar(redlower1, redlower2, redlower3), new Scalar(redupper1, redupper2, redupper3), contourMat);

        }

        //extracts yellow (for poles)
        //Core.extractChannel(generalMat, contourMat, 0);
        // detects edges
        //Imgproc.Canny(contourMat, contourMat, 100, 300);


        //contours
        Imgproc.findContours(contourMat, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
//        Imgproc.drawContours(generalMat, contours, -1, new Scalar(0, 255, 255), 2/*, Imgproc.LINE_8,
//                hierarchy, 2, new Point()*/);


        // loop through contours to find max
        int indexOfMax = 0;
        boundArea = 0;
        for (int i = 0; i < contours.size(); i++) {
            double area = Imgproc.contourArea(contours.get(i));
            if (area > boundArea) {
                boundArea = area;
                indexOfMax = i;
            }
        }


        if (contours.size() != 0) {
            //draws largest contour
            Imgproc.drawContours(generalMat, contours, indexOfMax, new Scalar(0, 255, 255), 2/*, Imgproc.LINE_8,
            hierarchy, 2, new Point()*/);

            // bounding box (approximate width)
            Rect boundRect = Imgproc.boundingRect(contours.get(indexOfMax));
            perWidth = boundRect.width;
            Imgproc.rectangle(generalMat, boundRect, new Scalar (20,20,20), 1);

    /*
            int x,y,w,h = contourMat.boundingRect(contours.get(indexOfMax));
            perWidth =
    */

            //gets the moments of the contour in question ...idk how
            M = Imgproc.moments(contours.get(indexOfMax));
            // gets x and y of centroid
            cX = (int)(M.get_m10() / M.get_m00());
            cY = (int) (M.get_m01() / M.get_m00());

    /*
            // another method of finding center of contour
            Rect boundRect = Imgproc.boundingRect(contour);
            double centerX = boundRect.x + (boundRect.width / 2)
            double centerY = boundRect.y + (boundRect.height / 2)
    */
            // gets points of the main contour
            mainContour = contours.get(indexOfMax).toArray();
//            //telemetry.addData("bottom line 1", mainContour[mainContour.length - 34].x +"  "+ mainContour[mainContour.length - 34].y);
//            //telemetry.addData("bottom line 2", mainContour[mainContour.length - 35].x +"  "+ mainContour[mainContour.length - 35].y);
//            //telemetry.update();
//            if (mainContour.length > 6) {
//                loopSpecificPoints = new Point[]{mainContour[(int) (mainContour.length / 10)], mainContour[(int) (mainContour.length / 9)], mainContour[(int) (mainContour.length / 8)], mainContour[(int) (mainContour.length / 6)], mainContour[(int) (mainContour.length / 4)]};
//                // loop to get indexes for more looping :) --> adds needed points to arrays
//                for (int i = mainContour.length - 1; i >= mainContour.length / 2; i--) {
//                    loggingString += (mainContour[i].toString() + "\n");
//                    for (int j = 0; j < loopSpecificPoints.length; j++) {
//                        if (mainContour[i].y == loopSpecificPoints[j].y) {
//                            difference += (Math.abs(mainContour[i].x - loopSpecificPoints[j].x));
//                            loggingString += ("coordinate - value: " + mainContour[i].x + ", " + mainContour[i].y+ ", pre coordinate value: " + loopSpecificPoints[j].x + ", " + loopSpecificPoints[i].y + " total: " + difference + "\n");
//                        }
//                    }
//
//
//                    /*
//                    if (i >= 5) {
//                        num5Prev = mainContour[i - 5].x;
//                        num4Prev = mainContour[i - 4].x;
//                        num3Prev = mainContour[i - 3].x;
//                        num2Prev = mainContour[i - 2].x;
//                        num1Prev = mainContour[i - 1].x;
//                        numCurrent = mainContour[i].x;
//
//
//                        if (num5Prev < num4Prev && num4Prev == num3Prev && num3Prev == num2Prev && num2Prev == num1Prev && num1Prev == numCurrent) {
//                            // left top (last element / index of arraylist)
//                            telemetry.addData("leftContour.add(mainContour[i - 4].x);", mainContour[i - 4].x);
//                            telemetry.update();
//                            leftContour.add(mainContour[i - 4].x);
//                            toggle = false;
//                        } else if (num5Prev == num4Prev && num4Prev == num3Prev && num3Prev == num2Prev && num2Prev == num1Prev && num1Prev != numCurrent) {
//                            // right top (first element / index of the right arraylist)
//                            telemetry.addData("rightContour.add(mainContour[i - 1].x);", mainContour[i - 1].x);
//                            telemetry.update();
//                            rightContour.add(mainContour[i - 1].x);
//                            toggle2 = true;
//                            toggle = false;
//                        }
//
//                        if (rightContour != null) {
//
//                            if (rightContour.size() > 3 && leftContour.size() > 3 && rightContour.size() == leftContour.size()) {
//                                telemetry.addData("break", mainContour[i - 1].x);
//                                telemetry.update();
//                                break;
//                            }
//                        }
//
//                        if (toggle2 && (rightContour.size() != leftContour.size())) {
//                            rightContour.add(mainContour[i].x);
//                            telemetry.addData("toggle2", mainContour[i - 1].x);
//                            telemetry.update();
//                        }
//                        if (toggle) {
//                            leftContour.add(mainContour[i - 5].x);
////                            telemetry.addData("toggle", mainContour[i - 1].x);
//                            telemetry.update();
//                        }
//                    }
//                    */
//
//                }
//
//                /*
//                // compare the 2 lists (average distance)
//                for (int i = 0; i < rightContour.size(); i++) {
//                    leftOfContour += leftContour.get(i);
//                    rightOfContour += rightContour.get(i);
//
//                }
//                */
//                 //*/
//                //perWidth = Math.abs(leftOfContour - rightOfContour) / (rightContour.size());
//                perWidth = difference / loopSpecificPoints.length;

    /*
                try {
                    // loops through the x values for the contour
                    for (int i = 0; i < mainContour.length / 2 - 30; i++) {
                        leftOfContour += mainContour[i].x;
                    }
                    for (int i = mainContour.length - 1; i >= mainContour.length / 2; i--) {
                        rightOfContour += mainContour[i].x;
                    }
                    //gets the average distance between each x point (that's why it's divided by one half of
                    //the values it looped for; the difference means every 2 instead of the average of every 1)
                    perWidth = Math.abs(leftOfContour - rightOfContour) / (mainContour.length / 2);
                } catch (Exception e){
                    telemetry.addData("error", e);
                }
    */

            //draws circle of centroid
            Imgproc.circle(generalMat, new Point(cX, cY), 1, new Scalar(255, 49, 0, 255), 2);
            focalLength = (311.5384615 + 311.5384615 + 311.9711538) / 3.0;
            distance = distanceFromPole(1.04, focalLength, perWidth) * Math.cos(38.6);

            telemetry.addData("index", indexOfMax);
            telemetry.addData("cX", cX);
            telemetry.addData("cY", cY);
            telemetry.addData("area", boundArea);
//                telemetry.addData("array", mainContour[0].x + " " + mainContour[0].y);
//                telemetry.addData("array", mainContour[1].x + " " + mainContour[1].y);
//                telemetry.addData("leftcontour size", leftContour.size());
//                telemetry.addData("rightcontour size", rightContour.size());
//                telemetry.addData("maincontour", mainContour[1].y);
//
            loggingString += ("---------------------------------------------------------" + "\n");
            writeLoggerToFile();

            telemetry.update();
//            }
        }
        return generalMat;
    }

    // method for determining distance of camera to object
    // knownWidth = the actual width of the object "irl"
    // focalLength = idk
    // perceivedWidth = what the width looks like in pixels from the cam
    // F = (P * Distance) / W
    //must start by calculating the focalLength by using a known distance
    public double distanceFromPole(double knownWidth, double focalLength, double perWidth) {
        return (knownWidth * focalLength) / perWidth;
    }

    public double getFocalLength(double knownWidth, double dist, double perWidth) {
        return (perWidth * dist) / knownWidth;
    }
    // perWidth = 22.0; (pixels)
    // dist = 18.5; (inches)
    // focal = 391.3461538461538

    // 34.5 // 20.5 // 15
    // 12 // 18 // 24
    // 398.0769230769231 // 354.8076923076923 // 346.1538461538462

    // actual width of pole 1.04 in.
    // https://pyimagesearch.com/2015/01/19/find-distance-camera-objectmarker-using-python-opencv/


    public int getcX() {
        return cX;
    }

    public int getcY() {
        return cY;
    }

    public double getDistance() {
        return distance;
    }

    public double getBoundArea() {
        return boundArea;
    }

    // logs string into file
    public void writeLoggerToFile(){
        try{
            PrintStream toFile = new PrintStream(loggingFile);
            toFile.println(loggingString);
        }catch(FileNotFoundException e){
            e.printStackTrace();
        }
    }
}
