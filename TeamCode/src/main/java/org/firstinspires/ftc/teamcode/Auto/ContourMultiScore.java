package org.firstinspires.ftc.teamcode.Auto;


import org.firstinspires.ftc.robotcore.external.Telemetry;
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

public class ContourMultiScore extends OpenCvPipeline {
    // dimensions of camera image: 320, 176

    // defining vars
    private Mat contourMat = new Mat(), generalMat = new Mat(), highestPole = new Mat(), hierarchy = new Mat();
    Moments M;
    private int cX;
    private  int cY;
    private List<MatOfPoint> contours = new ArrayList<>();
    double knownWidth, focalLength, perWidth;
    Telemetry telemetry;

    public ContourMultiScore (Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat     input) {
        knownWidth = 1.04; //inches



        generalMat = input.clone();
        //input.copyTo(generalMat);

        if(input.empty()) {
            return input;
        }
        // image tuning
        Imgproc.GaussianBlur(generalMat, generalMat, new Size(5, 5), 0);
        Imgproc.erode(generalMat, generalMat, new Mat(), new Point(-1, -1), 2);
        Imgproc.dilate(generalMat, generalMat, new Mat(), new Point(-1, -1), 2);
        Imgproc.cvtColor(generalMat, contourMat, Imgproc.COLOR_RGB2YCrCb);
        Core.inRange(contourMat, new Scalar(255, 245, 149), new Scalar(212, 191, 0), contourMat);

        //extracts yellow (for poles)
        //Core.extractChannel(generalMat, contourMat, 0);
        // detects edges
        //Imgproc.Canny(contourMat, contourMat, 100, 300);


        //contours
        Imgproc.findContours(contourMat, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        Imgproc.drawContours(generalMat, contours, -1, new Scalar(0, 255, 255), 2/*, Imgproc.LINE_8,
                hierarchy, 2, new Point()*/);
/*
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



        //draws largest contour
        Imgproc.drawContours(contourMat, contours, indexOfMax, new Scalar(255, 255, 0), 2, Imgproc.LINE_8,
                contourMat, 2, new Point());
        Rect boundRect = Imgproc.boundingRect(contours.get(indexOfMax));
        perWidth = boundRect.width;

/*
        int x,y,w,h = contourMat.boundingRect(contours.get(indexOfMax));
        perWidth =
*/
/*
        //gets the moments of the contour in question ...idk how
        M = Imgproc.moments(contours.get(indexOfMax));
        // gets x and y of centroid
        cX = (int)(M.get_m10() / M.get_m00());
        cY = (int) (M.get_m01() / M.get_m00());
*/
/*
        // another method of finding center of contour
        Rect boundRect = Imgproc.boundingRect(contour);
        double centerX = boundRect.x + (boundRect.width / 2)
        double centerY = boundRect.y + (boundRect.height / 2)
*/
        //draws circle of centroid
//        Imgproc.circle(contourMat, new Point(cX, cY), 4, new Scalar(255,49,0,255), 4);
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


    // actual width of pole 1.04 in.
    // https://pyimagesearch.com/2015/01/19/find-distance-camera-objectmarker-using-python-opencv/
}
