package teamcode.Competition.Pipeline.MecanumPipeline;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import teamcode.common.AbstractOpMode;

public class MecanumBarcodePipeline3 extends OpenCvPipeline {

    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);

    static final int REGION_WIDTH = 40;
    static final int REGION_HEIGHT = 80;

    static final double SIZE = REGION_WIDTH * REGION_HEIGHT;

    static Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(108, 85);
    static Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(270, 85);

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

    @Override
    public Mat processFrame(Mat mat) {
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_BGR2RGB);
        Core.inRange(mat, new Scalar(80, 80, 125), new Scalar(120, 120, 255), mask);

        region1 = mask.submat(new Rect(region1_pointA, region1_pointB));
        region2 = mask.submat(new Rect(region2_pointA, region2_pointB));

        Imgproc.rectangle(
                mat,
                region1_pointA,
                region1_pointB,
                BLUE,
                2
        );
        Imgproc.rectangle(
                mat,
                region2_pointA,
                region2_pointB,
                BLUE,
                2
        );

        AbstractOpMode.currentOpMode().telemetry.addData("R1 - WHITE PX:", Core.countNonZero(region1));
        AbstractOpMode.currentOpMode().telemetry.addData("R1 - WHITE/BLACK PX:", Math.round((Core.countNonZero(region1) / SIZE) * 100.0) / 100.0);
        AbstractOpMode.currentOpMode().telemetry.addData("R2 - WHITE PX:", Core.countNonZero(region2));
        AbstractOpMode.currentOpMode().telemetry.addData("R2 - WHITE/BLACK PX:", Math.round((Core.countNonZero(region2) / SIZE) * 100.0) / 100.0);
        AbstractOpMode.currentOpMode().telemetry.update();
        return mat;
    }
}
