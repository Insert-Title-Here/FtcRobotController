package org.firstinspires.ftc.teamcode.Auto;


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
public class ContourMultiScore extends OpenCvPipeline {
    public boolean park = true;

    /*
   YELLOW  = Parking Left
   CYAN    = Parking Middle
   MAGENTA = Parking Right
    */
    // Mat defined/instantiated, percents (for each color) instantiated
    private Mat yelMat = new Mat(), cyaMat = new Mat(), magMat = new Mat(), changed = new Mat(), original = new Mat();
    private double yelPercent, cyaPercent, magPercent;

    private static int box_x = 111;
    private static int box_y = 101;

    // top left point of submat (original 320, 176)
    private static Point BOX_TOPLEFT = new Point(box_x,box_y); // 175, 150

    // width and height of submat
    public static int BOX_WIDTH = 23;
    public static int BOX_HEIGHT = -38;

    enum ParkingPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    // Running variable storing the parking position
    private volatile DetectionAlgorithmTest.ParkingPosition position = DetectionAlgorithmTest.ParkingPosition.LEFT;

    // submat to center cone
    Point box_top_left = new Point(
            BOX_TOPLEFT.x,
            BOX_TOPLEFT.y);
    Point box_bottom_right = new Point(
            BOX_TOPLEFT.x + BOX_WIDTH,
            BOX_TOPLEFT.y + BOX_HEIGHT);

    // Lower and upper boundaries for colors -> YCrCb
    private static final Scalar
            lower_yellow_bounds  = new Scalar(convertToY(200, 200, 0)),
            upper_yellow_bounds  = new Scalar(convertToY(255, 255, 130)),
            lower_cyan_bounds    = new Scalar(convertToCb(0, 200, 200)),
            upper_cyan_bounds    = new Scalar(convertToCb(150, 255, 255)),
            lower_magenta_bounds = new Scalar(convertToCr(170, 0, 170)),
            upper_magenta_bounds = new Scalar(convertToCr(255, 60, 255));

    // Color definitions -> RGB
    private final Scalar
            YELLOW  = new Scalar(255, 255, 0),
            CYAN    = new Scalar(0, 255, 255),
            MAGENTA = new Scalar(255, 0, 255);



    // dimensions of camera image: 320, 176

    // defining vars
    private Mat contourMat = new Mat(), generalMat = new Mat(), highestPole = new Mat(), hierarchy = new Mat();
    Moments M;
    private int cX;
    private  int cY;
    private List<MatOfPoint> contours = new ArrayList<>();
    private Point[] mainContour;
    private ArrayList<Double> leftContour, rightContour;
    private Point[] loopSpecificPoints;
    private double knownWidth, focalLength, perWidth, distance, leftOfContour, rightOfContour, difference = 0;
    // for points on contour
    double numCurrent, num1Prev, num2Prev, num3Prev, num4Prev, num5Prev = 0;
    boolean toggle, toggle2;

    // creates/accesses file
    File loggingFile = AppUtil.getInstance().getSettingsFile("telemetry.txt");
    // holds data
    public String loggingString;

    Telemetry telemetry;

    /* make stuff public static  */
    public static int lower1 = 50;
    public static int lower2 = 50;
    public static int lower3 = 10;
    public static int upper1 = 255;
    public static int upper2 = 255;
    public static int upper3 = 105; //value was 80 when servo was perpendicular to the ground
    public static double widthRemoveConstant = 3.1;


    public ContourMultiScore (Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        if (park) {
            Mat befChange = new Mat();

            input.copyTo(original);

            if(original.empty()) {
                return input;
            }
            // cyan magenta yellow

        /* colors (scalars):
            magenta -> new Scalar(255, 0, 255)
            yellow -> new Scalar(255, 255, 0)
            cyan -> new Scalar(0, 255, 255)
         */
            changed = original.submat(new Rect(box_top_left, box_bottom_right));



            //Core.extractChannel(changed, changed, 1);

            Imgproc.GaussianBlur(changed, changed, new Size(5,5), 0);
            Imgproc.erode(changed, changed, new Mat(), new Point(-1, -1), 2);
            Imgproc.dilate(changed, changed, new Mat(), new Point(-1, -1), 2);
            Imgproc.cvtColor(changed, changed, Imgproc.COLOR_RGB2YCrCb);
            befChange = changed.submat(new Rect(new Point(11, 11), new Point(12, 12)));
            //Y -> brightness, Cr -> red - brightness, Cb -> blue - brightness
            Core.extractChannel(changed, yelMat, 0);
            Core.extractChannel(changed, cyaMat, 2);
            Core.extractChannel(changed, magMat, 1);
//        // Apply Morphology
//        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
//        Imgproc.morphologyEx(changed, changed, Imgproc.MORPH_CLOSE, kernel);
//


            /* submatrices
            Mat pixel_section = original.submat(rowStart, rowEnd, colStart, colEnd)l

             */

            // https://sistenix.com/rgb2ycbcr.html -> convert between rgb and ycbcr
            // yellow 190
//        Core.inRange(yelMat, lower_yellow_bounds, upper_yellow_bounds, yelMat);
            Core.inRange(yelMat, new Scalar(160), new Scalar(205), yelMat);
            // cyan 183
//        Core.inRange(cyaMat, lower_cyan_bounds, upper_cyan_bounds, cyaMat);
            Core.inRange(cyaMat, new Scalar(170), new Scalar(194), cyaMat);
            // magenta 186
//        Core.inRange(magMat, lower_magenta_bounds, upper_magenta_bounds, magMat);
            Core.inRange(magMat, new Scalar(161), new Scalar(195), magMat);

//        befChange.convertTo(befChange, CvType.CV_64FC3);
//        byte buff[] = new byte[ (int) (befChange.total() * befChange.channels())];
//        byte buff2[] = new byte[3];
            // percent "abundance" for each color
            yelPercent = Core.countNonZero(yelMat);
            cyaPercent = Core.countNonZero(cyaMat);
            magPercent = Core.countNonZero(magMat);
            telemetry.addData("yelPercent", yelPercent);
            telemetry.addData("cyaPercent", cyaPercent);
            telemetry.addData("magPercent", magPercent);
//        telemetry.addData("cyanColor", changed.get(0, 0,buff2));
            telemetry.addData("cyanColor", Core.sumElems(befChange).val[2]);
            telemetry.addData("magColor", Core.sumElems(befChange).val[1]);
            telemetry.addData("yelColor", Core.sumElems(befChange).val[0]);
            telemetry.update();

            // decides parking position, highlights margin according to greatest abundance color
            if (yelPercent > cyaPercent) {
                if (yelPercent > magPercent) {
                    // yellow greatest, position left
                    position = DetectionAlgorithmTest.ParkingPosition.LEFT;
                    //telemetry.addData("park position", position);
                    Imgproc.rectangle(original, new Rect(box_top_left, box_bottom_right), YELLOW, 2);
                } else {
                    // magenta greatest, position right
                    position = DetectionAlgorithmTest.ParkingPosition.RIGHT;
                    //telemetry.addData("park position", position);
                    Imgproc.rectangle(original, new Rect(box_top_left, box_bottom_right), MAGENTA, 2);
                }
            } else if(cyaPercent > magPercent) {
                // cyan greatest, position center
                position = DetectionAlgorithmTest.ParkingPosition.CENTER;
                //telemetry.addData("park position", position);
                Imgproc.rectangle(original, new Rect(box_top_left, box_bottom_right), CYAN, 2);
            } else {

                // magenta greatest, position right
                position = DetectionAlgorithmTest.ParkingPosition.RIGHT;
                //telemetry.addData("park position", position);
                Imgproc.rectangle(original, new Rect(box_top_left, box_bottom_right), MAGENTA, 2);

            }
            telemetry.update();

            // Memory cleanup
            changed.release();
            //original.release();
            yelMat.release();
            cyaMat.release();
            magMat.release();

            return original;
        } else {


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

            generalMat = input.clone();
            //input.copyTo(generalMat);

            if (input.empty()) {
                return input;
            }
            // image tuning
            Imgproc.GaussianBlur(generalMat, contourMat, new Size(5, 5), 0);
            Imgproc.erode(contourMat, contourMat, new Mat(), new Point(-1, -1), 2);
            Imgproc.dilate(contourMat, contourMat, new Mat(), new Point(-1, -1), 2);
            Imgproc.cvtColor(contourMat, contourMat, Imgproc.COLOR_RGB2YCrCb);
            //        Core.extractChannel(contourMat, contourMat, 0);
            Core.inRange(contourMat, new Scalar(lower1, lower2, lower3), new Scalar(upper1, upper2, upper3), contourMat);

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
            double largestArea = 0;
            for (int i = 0; i < contours.size(); i++) {
                double area = Imgproc.contourArea(contours.get(i));
                if (area > largestArea) {
                    largestArea = area;
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
                Imgproc.rectangle(generalMat, boundRect, new Scalar(20, 20, 20), 1);

        /*
                int x,y,w,h = contourMat.boundingRect(contours.get(indexOfMax));
                perWidth =
        */

                //gets the moments of the contour in question ...idk how
                M = Imgproc.moments(contours.get(indexOfMax));
                // gets x and y of centroid
                cX = (int) (M.get_m10() / M.get_m00());
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
                for (int i = mainContour.length - 1; i >= 0; i--) {
                    loggingString += (mainContour[i].toString() + "\n");
                    if (Math.abs(mainContour[4].x - mainContour[i].x) > 6 && mainContour[i].y > 3) {
                        perWidth = Math.abs(mainContour[4].x - mainContour[i].x);
                        loggingString += ("perWidth " + perWidth + "\n");
                        break;
                    }
                }

                //draws circle of centroid
                Imgproc.circle(generalMat, new Point(cX, cY), 1, new Scalar(255, 49, 0, 255), 2);
                focalLength = (311.5384615 + 311.5384615 + 311.9711538) / 3.0;
                distance = distanceFromPole(1.04, focalLength, perWidth) * Math.cos(38.6);

                telemetry.addData("index", indexOfMax);
                telemetry.addData("cX", cX);
                telemetry.addData("cY", cY);
                telemetry.addData("width", perWidth);
                telemetry.addData("width v.2", boundRect.width);
                telemetry.addData("distanceInches", distance);
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

    // logs string into file
    public void writeLoggerToFile(){
        try{
            PrintStream toFile = new PrintStream(loggingFile);
            toFile.println(loggingString);
        }catch(FileNotFoundException e){
            e.printStackTrace();
        }
    }


    public static double convertToY(int r, int g, int b) {
        return 16 + (65.738 * r + 129.057 * g + 25.064 * b) / 256;
    }

    public static double convertToCb(int r, int g, int b) {
        return 128 - (37.945 * r - 74.494 * g + 112.439 * b) / 256;
    }

    public static double convertToCr(int r, int g, int b) {
        return 128 + (112.439 * r - 94.154 * g - 18.285 * b) / 256;
    }

    // Returns an enum being the current position where the robot will park
    public DetectionAlgorithmTest.ParkingPosition getPosition() {
        return position;
    }
}
