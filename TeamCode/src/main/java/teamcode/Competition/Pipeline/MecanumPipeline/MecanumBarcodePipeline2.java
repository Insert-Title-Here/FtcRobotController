package teamcode.Competition.Pipeline.MecanumPipeline;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import teamcode.common.AbstractOpMode;

public class MecanumBarcodePipeline2 extends OpenCvPipeline {
    public enum BarcodePosition {
        LEFT,
        CENTER,
        RIGHT
    }

    public enum Side {
        RED,
        BLUE
    }

    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);

    // InRange is BGR to these Scalars are in BGR
    static final Scalar lowerRed = new Scalar(50, 50, 0);
    static final Scalar upperRed = new Scalar(255, 255, 255);

    static Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(123, 150);
    static Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(270, 150);

    static final int REGION_WIDTH = 40;
    static final int REGION_HEIGHT = 80;

    static final double SIZE = REGION_WIDTH * REGION_HEIGHT;

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

    Mat mask = new Mat();
    Mat region1 = new Mat();
    Mat region2 = new Mat();

    volatile BarcodePosition position = BarcodePosition.RIGHT;
    volatile Side side = Side.RED;

    void rangeProcess(Mat input) {
        Core.inRange(input, new Scalar(85, 85, 135), new Scalar(255, 120, 110), mask);
    }

    @Override
    public void init(Mat frame) {
        rangeProcess(frame);

        region1 = mask.submat(new Rect(region1_pointA, region1_pointB));
        region2 = mask.submat(new Rect(region2_pointA, region2_pointB));
    }

    @Override
    public Mat processFrame(Mat input) {
        rangeProcess(input);

        REGION1_TOPLEFT_ANCHOR_POINT = new Point(108, 85);
        REGION2_TOPLEFT_ANCHOR_POINT = new Point(270, 85);

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

        /*region1 = bMat.submat(new Rect(region1_pointA, region1_pointB));
        region2 = bMat.submat(new Rect(region2_pointA, region2_pointB));

        avg1 = (int) Core.mean(region1).val[0];
        avg2 = (int) Core.mean(region2).val[0];
        int min = Math.min(avg1, avg2);*/

        // processing
        //AbstractOpMode.currentOpMode().telemetry.addData("non-zero:", Core.countNonZero(input));

        //Imgproc.erode(mask, mask, new Mat(), new Point(-1, -1), 4);
        //Imgproc.dilate(mask, mask, new Mat(), new Point(-1, -1), 2);

        region1 = mask.submat(new Rect(region1_pointA, region1_pointB));
        region2 = mask.submat(new Rect(region2_pointA, region2_pointB));

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

        AbstractOpMode.currentOpMode().telemetry.addData("R1 - AVGR", Core.mean(input, mask));
        AbstractOpMode.currentOpMode().telemetry.addData("R1 - WHITE PX:", Core.countNonZero(region1));
        AbstractOpMode.currentOpMode().telemetry.addData("R1 - WHITE/BLACK PX:", Math.round((Core.countNonZero(region1) / SIZE) * 100.0) / 100.0);
        AbstractOpMode.currentOpMode().telemetry.addData("R2 - WHITE PX:", Core.countNonZero(region2));
        AbstractOpMode.currentOpMode().telemetry.addData("R2 - WHITE/BLACK PX:", Math.round((Core.countNonZero(region2) / SIZE) * 100.0) / 100.0);
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
