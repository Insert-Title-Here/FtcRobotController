package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class TestingBarcodePipeline extends OpenCvPipeline {

    public enum BarcodePosition{
        LEFT,
        CENTER,
        RIGHT
    }

    static final Scalar RED = new Scalar(255, 0, 0);
    static final Scalar YELLOW = new Scalar(234, 170, 0);
    static final int calibratedRange = 90;
    static int currentMaxValue = 0;

    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(70, 190);
    static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(140, 190);
    static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(210, 190);

    static final int REGION_WIDTH = 40;
    static final int REGION_HEIGHT = 50;

    Point region1_pointA = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x,
            REGION1_TOPLEFT_ANCHOR_POINT.y
    ) ;

    Point region1_pointB = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT
    );

    Point region2_pointA = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x,
            REGION2_TOPLEFT_ANCHOR_POINT.y
    ) ;

    Point region2_pointB = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT
    );

    Point region3_pointA = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x,
            REGION3_TOPLEFT_ANCHOR_POINT.y
    ) ;

    Point region3_pointB = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT
    );

    Mat region1_RGB, region2_RGB, region3_RGB;

    Mat RGB = new Mat();
    Mat B = new Mat();
    //Mat workingMatrix = new Mat();

    int avg1, avg2, avg3;

    volatile BarcodePosition position = BarcodePosition.CENTER;

    void inputToHSV(Mat input){
        Imgproc.cvtColor(input, RGB, Imgproc.COLOR_RGB2BGR);
        Core.extractChannel(RGB, B, 0);
    }



    @Override
    public void init(Mat frame) {
        inputToHSV(frame);

        region1_RGB = B.submat(new Rect(region1_pointA, region1_pointB));
        region2_RGB = B.submat(new Rect(region2_pointA, region2_pointB));
        region3_RGB = B.submat(new Rect(region3_pointA, region3_pointB));


    }

    @Override
    public Mat processFrame(Mat input) {
        inputToHSV(input);

        region1_RGB = B.submat(new Rect(region1_pointA, region1_pointB));
        region2_RGB = B.submat(new Rect(region2_pointA, region2_pointB));
        region3_RGB = B.submat(new Rect(region3_pointA, region3_pointB));

        avg1 = (int) Core.mean(region1_RGB).val[0];
        avg2 = (int) Core.mean(region2_RGB).val[0];
        avg3 = (int) Core.mean(region3_RGB).val[0];

        Imgproc.rectangle(
                input,
                region1_pointA,
                region1_pointB,
                YELLOW,
                2
        );

        Imgproc.rectangle(
                input,
                region2_pointA,
                region2_pointB,
                YELLOW,
                2
        );

        Imgproc.rectangle(
                input,
                region3_pointA,
                region3_pointB,
                YELLOW,
                2
        );

        //int max = Math.max(Math.max(avg1, avg2), avg3);
        //currentMaxValue = max;

        if(avg1 < calibratedRange){
            Imgproc.rectangle(
                    input,
                    region1_pointA,
                    region1_pointB,
                    RED,
                    2
            );
            position = BarcodePosition.LEFT;
        }else if(avg3 < calibratedRange){
            Imgproc.rectangle(
                    input,
                    region3_pointA,
                    region3_pointB,
                    RED,
                    2
            );
            position = BarcodePosition.RIGHT;
        }else{
            Imgproc.rectangle(
                    input,
                    region2_pointA,
                    region2_pointB,
                    RED,
                    2
            );
            position = BarcodePosition.CENTER;
        }

        return input;






    }

    public BarcodePosition getPos(){return position;}

    public int getCurrentMaxValue(){return currentMaxValue;};
}
