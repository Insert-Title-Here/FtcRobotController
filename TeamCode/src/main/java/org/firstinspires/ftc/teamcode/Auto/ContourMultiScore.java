package org.firstinspires.ftc.teamcode.Auto;


import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
public class ContourMultiScore extends OpenCvPipeline {
    // dimensions of camera image: 320, 176

    // defining vars
    private Mat contourMat = new Mat(), generalMat = new Mat(), highestPole = new Mat(), hierarchy = new Mat();
    Moments M;
    private int cX;
    private  int cY;
    private List<MatOfPoint> contours = new ArrayList<>();
    private Point[] mainContour;
    double knownWidth, focalLength, perWidth, distance, leftOfContour, rightOfContour = 0;
    Telemetry telemetry;

    /* make stuff public static  */
    public static int lower1 = 50;
    public static int lower2 = 50;
    public static int lower3 = 10;
    public static int upper1 = 255;
    public static int upper2 = 255;
    public static int upper3 = 80;


    public ContourMultiScore (Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        knownWidth = 1.04; //inches
        contours = new ArrayList<>();
        // 43, 157, 166

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

            Rect boundRect = Imgproc.boundingRect(contours.get(indexOfMax));
            perWidth = boundRect.width;

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
            try {
                // loops through the x values for the contour
                for (int i = 0; i < mainContour.length / 2; i++) {
                    leftOfContour += mainContour[i].x;
                }
                for (int i = mainContour.length; i > mainContour.length / 2; i--) {
                    rightOfContour += mainContour[i].x;
                }
                //gets the average distance between each x point (that's why it's divided by one half of
                //the values it looped for; the difference means every 2 instead of the average of every 1)
                perWidth = Math.abs(leftOfContour - rightOfContour) / (mainContour.length / 2);
            } catch (Exception e){
                telemetry.addData("error", e);
            }
            //draws circle of centroid
            Imgproc.circle(contourMat, new Point(cX, cY), 4, new Scalar(255,49,0,255), 4);
            focalLength = (398.0769230769231 + 354.8076923076923 + 346.1538461538462 + 391.3461538461538) / 4.0;
            distance = distanceFromPole(1.04, focalLength, perWidth);

            telemetry.addData("index", indexOfMax);
            telemetry.addData("cX", cX);
            telemetry.addData("cY", cY);
            telemetry.addData("width", perWidth);
            telemetry.addData("distanceInches", distance - 0.75);
            telemetry.addData("array", mainContour[0].x +" "+ mainContour[0].y);


            telemetry.update();
        }
/*


        //movement if robot isn't centered to pole
        if (cX < 160) {
            while (cX < 160) {
                // turn to the right
            }
        } else if (cX > 160) {
            while (cX > 160) {
                // turn to the left
            }
        } else { // cX must equal to 160
            // move forward until close enough
        }


 */
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
}
