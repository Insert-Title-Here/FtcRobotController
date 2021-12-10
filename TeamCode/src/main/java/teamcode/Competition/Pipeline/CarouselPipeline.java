package teamcode.Competition.Pipeline;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import teamcode.common.AbstractOpMode;

public class CarouselPipeline extends OpenCvPipeline {
    public enum BarcodePosition {
        LEFT,
        CENTER,
        RIGHT
    }

    public enum Side {

        RED, BLUE;
    }


    // define col constants
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);

    // get anchor points for each region
    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(0, 120);
    static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(65, 120);
    static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(210, 120);
    static final int REGION_WIDTH = 40;
    static final int REGION_HEIGHT = 40;

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
    Point region3_pointA = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x,
            REGION3_TOPLEFT_ANCHOR_POINT.y);
    Point region3_pointB = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    // Create vars
    Mat region1_Cb, region2_Cb, region3_Cb;
    //    Mat YCrCb = new Mat();
//    Mat Cb = new Mat();
    Mat RGB = new Mat();
    Mat B = new Mat();
    int avg1, avg2, avg3;

    // create pos var, with vol tag due to the var changing at runtime
    volatile BarcodePosition position = BarcodePosition.RIGHT;
    volatile Side side = Side.BLUE;


    // converts rgb frame to ycrcb, extracts cb channel
//    void inputToCb(Mat input) {
//        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
//        Core.extractChannel(YCrCb, Cb, 2);
//    }

    void inputToB(Mat input) {
        Imgproc.cvtColor(input, RGB, Imgproc.COLOR_RGB2BGR);
        Core.extractChannel(RGB, B, 0);
    }

    @Override
    public void init(Mat frame) {
        inputToB(frame);

        region1_Cb = B.submat(new Rect(region1_pointA, region1_pointB));
        region2_Cb = B.submat(new Rect(region2_pointA, region2_pointB));
        region3_Cb = B.submat(new Rect(region3_pointA, region3_pointB));
    }

    @Override
    public Mat processFrame(Mat input) {
        inputToB(input);

        region1_Cb = B.submat(new Rect(region1_pointA, region1_pointB)); //left region
        region2_Cb = B.submat(new Rect(region2_pointA, region2_pointB)); //center region
        region3_Cb = B.submat(new Rect(region3_pointA, region3_pointB)); //right region

        avg1 = (int) Core.mean(region1_Cb).val[0];
        avg2 = (int) Core.mean(region2_Cb).val[0];
        avg3 = (int) Core.mean(region3_Cb).val[0];

//        Imgproc.rectangle(
//                input,
//                region1_pointA,
//                region1_pointB,
//                BLUE,
//                2
//        );

        Imgproc.rectangle(
                input,
                region2_pointA,
                region2_pointB,
                BLUE,
                2
        );

        Imgproc.rectangle(
                input,
                region3_pointA,
                region3_pointB,
                BLUE,
                2
        );

        int min;
        double side1;
        double side2;
        //min = Math.min(Math.min(avg1, avg2), avg3);

        min = Math.min(avg2, avg3);
        side1 = avg2;
        side2 = avg3;


        if(Math.abs(side1 - side2) < 20){
            position = BarcodePosition.LEFT;
        }else if (min == side1) {

            Imgproc.rectangle(
                    input,
                    region2_pointA,
                    region2_pointB,
                    GREEN,
                    2
            );
            if (side == Side.BLUE) {
                position = BarcodePosition.RIGHT;
            } else {
                position = BarcodePosition.CENTER;
            }
        } else if (min == side2) {

            Imgproc.rectangle(
                    input,
                    region3_pointA,
                    region3_pointB,
                    GREEN,
                    2
            );
            if(side == Side.BLUE){
                position = BarcodePosition.CENTER;
            }else {
                position = BarcodePosition.RIGHT;
            }
        }

        AbstractOpMode.currentOpMode().telemetry.addData("avg1", avg1);
        AbstractOpMode.currentOpMode().telemetry.addData("avg2", avg2);
        AbstractOpMode.currentOpMode().telemetry.addData("avg3", avg3);
        AbstractOpMode.currentOpMode().telemetry.addData("position", position);
        AbstractOpMode.currentOpMode().telemetry.update();
        return input;
    }

    public BarcodePosition getPos() {
        return position;
    }

    public void setSide(Side side) {
        this.side = side;
    }

    public Side getSide() {
        return side;
    }
}
