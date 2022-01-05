package org.firstinspires.ftc.teamcode.MecanumCode.Auto.Vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class BarcodePipelineWarehouseRed extends OpenCvPipeline{
    // define position enums
    public enum BarcodePosition
    {
        LEFT,
        CENTER,
        RIGHT
    }

    // define col constants
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);
    static final int calibratedRange = 90;
    static int currentMinValue = 0;

    // get anchor points for each region
    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(140, 190);
    static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(280, 190);
    static final int REGION_WIDTH = 40;
    static final int REGION_HEIGHT = 50;

    // define top left and bottom right region points
    Point region1_pointA = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x,
            REGION1_TOPLEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point region2_pointA = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x,
            REGION2_TOPLEFT_ANCHOR_POINT.y);
    Point region2_pointB = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    // Create vars
    Mat region1_Cb, region2_Cb;
    //    Mat YCrCb = new Mat();
//    Mat Cb = new Mat();
    Mat RGB = new Mat();
    Mat B = new Mat();
    int avg1, avg2;

    // create pos var, with vol tag due to the var changing at runtime
    volatile BarcodePosition position = BarcodePosition.CENTER;

    void inputToB(Mat input) {
        Imgproc.cvtColor(input, RGB, Imgproc.COLOR_RGB2BGR);
        Core.extractChannel(RGB, B, 0);
    }


    @Override
    public void init(Mat frame) {
        inputToB(frame);

        region1_Cb = B.submat(new Rect(region1_pointA, region1_pointB));
        region2_Cb = B.submat(new Rect(region2_pointA, region2_pointB));
    }

    @Override
    public Mat processFrame(Mat input) {
        inputToB(input);

        region1_Cb = B.submat(new Rect(region1_pointA, region1_pointB));
        region2_Cb = B.submat(new Rect(region2_pointA, region2_pointB));

        avg1 = (int) Core.mean(region1_Cb).val[0];
        avg2 = (int) Core.mean(region2_Cb).val[0];

        Imgproc.rectangle(
                input,
                region1_pointA,
                region1_pointB,
                BLUE,
                2
        );

        Imgproc.rectangle(
                input,
                region2_pointA,
                region2_pointB,
                BLUE,
                2
        );



        int min = Math.min(avg1, avg2);
        currentMinValue = min;

        if (min >= calibratedRange) {
            position = BarcodePosition.LEFT;
        } else if (min == avg1) {
            Imgproc.rectangle(
                    input,
                    region1_pointA,
                    region1_pointB,
                    GREEN,
                    2
            );
            position = BarcodePosition.CENTER;
        } else if (min == avg2) {
            Imgproc.rectangle(
                    input,
                    region2_pointA,
                    region2_pointB,
                    GREEN,
                    2
            );
            position = BarcodePosition.RIGHT;
        }

        return input;
    }

    public BarcodePosition getPos() {
        return position;
    }

    public int getCurrentMinValue() {
        return currentMinValue;
    }
}