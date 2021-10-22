package teamcode.Competition;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import teamcode.common.AbstractOpMode;


//TODO calibrate this pipeline
//potentially will need to rewrite this as this is a condensed Skystone pipeline,
//comparing the YCrCb index for detecting our obect may be sub optimal especially if it is something
//bright, run tests and adjust logic accordingly to be as consistent as possible
public class BarcodeReaderPipeline extends OpenCvPipeline{

    volatile BarcodePosition position;

    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);

    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(0,110);
    static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(160,110);
    static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(300,110);
    static final int REGION_WIDTH = 20;
    static final int REGION_HEIGHT = 40;


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
    Point region3_pointA = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x,
            REGION3_TOPLEFT_ANCHOR_POINT.y);
    Point region3_pointB = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    Mat region1_Cb, region2_Cb, region3_Cb;
    Mat YCrCb = new Mat();
    Mat Cb = new Mat();
    int avg1, avg2, avg3;

    @Override
    public void init(Mat firstFrame)
    {

        inputToCb(firstFrame);

        region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
        region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));
    }

    private void inputToCb(Mat input)
    {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 2);
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat workingMat = new Mat();
        input.copyTo(workingMat);
        if (workingMat.empty()) {
            return input;
        }
        Imgproc.cvtColor(workingMat, workingMat, Imgproc.COLOR_RGB2YCrCb);


        Mat left = workingMat.submat((int) REGION1_TOPLEFT_ANCHOR_POINT.x, (int) REGION1_TOPLEFT_ANCHOR_POINT.y, REGION_WIDTH, REGION_HEIGHT);
        Mat center = workingMat.submat((int) REGION2_TOPLEFT_ANCHOR_POINT.x, (int) REGION2_TOPLEFT_ANCHOR_POINT.y, REGION_WIDTH, REGION_HEIGHT);
        Mat right = workingMat.submat((int) REGION3_TOPLEFT_ANCHOR_POINT.x, (int) REGION3_TOPLEFT_ANCHOR_POINT.y, REGION_WIDTH, REGION_HEIGHT);

        Imgproc.rectangle(input, region1_pointA, region1_pointB, GREEN, 1);
        Imgproc.rectangle(input, region2_pointA, region2_pointB, GREEN, 1);
        Imgproc.rectangle(input, region3_pointA, region3_pointB, GREEN, 1);

        //inputToCb(input);


        avg1 = (int)Core.sumElems(left).val[2];
        avg2 = (int)Core.sumElems(center).val[2];
        avg3 = (int)Core.sumElems(right).val[2];

        AbstractOpMode.currentOpMode().telemetry.addData("average 1", avg1);
        AbstractOpMode.currentOpMode().telemetry.addData("average 2", avg2);
        AbstractOpMode.currentOpMode().telemetry.addData("average 3", avg3);
        AbstractOpMode.currentOpMode().telemetry.update();

        int max = Math.max(Math.max(avg1, avg2), avg3);

        if(max == avg1){
            position = BarcodePosition.LEFT;

        }else if(max == avg2){
            position = BarcodePosition.CENTER;

        }else if(max == avg3){
            position = BarcodePosition.RIGHT;
        }

        return input;
    }

    public enum BarcodePosition{
        LEFT, RIGHT, CENTER
    }

    public BarcodePosition getBarcodePosition(){
        return position;
    }
}