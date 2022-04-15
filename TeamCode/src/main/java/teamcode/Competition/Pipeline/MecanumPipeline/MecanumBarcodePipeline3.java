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
    static final Scalar WHITE = new Scalar(255, 255, 255);

    static final int REGION_WIDTH = 40;
    static final int REGION_HEIGHT = 80;

    static final double SIZE = REGION_WIDTH * REGION_HEIGHT;

    static Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(110, 95);
    static Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(253, 95);

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

    Mat filtered = new Mat();
    Mat mask = new Mat();
    Mat region1 = new Mat();
    Mat region2 = new Mat();
    Mat finalMat = new Mat();

    @Override
    public Mat processFrame(Mat mat) {
        Imgproc.cvtColor(mat, filtered, Imgproc.COLOR_RGB2BGR);
        Core.inRange(filtered, new Scalar(50, 90, 235), new Scalar(80, 117, 255), mask);
        //Imgproc.erode(mask, mask, new Mat(), new Point(-1, -1), 4);
        //Imgproc.dilate(mask, mask, new Mat(), new Point(-1, -1), 2);

        region1 = mask.submat(new Rect(region1_pointA, region1_pointB));
        region2 = mask.submat(new Rect(region2_pointA, region2_pointB));

        Imgproc.rectangle(
                mask,
                region1_pointA,
                region1_pointB,
                WHITE,
                2
        );
        Imgproc.rectangle(
                mask,
                region2_pointA,
                region2_pointB,
                WHITE,
                2
        );
        


        double[] color = mat.get(165, 130);
        Imgproc.circle(mask, new Point(130, 165), 5, new Scalar(color[0], color[1], color[2]), 2);
        double[] color2 = mat.get(165, 273);
        Imgproc.circle(mask, new Point(273, 165), 5, new Scalar(color2[0], color2[1], color2[2]), 2);
        //Imgproc.circle(mat, new Point(160, 120), 5, WHITE, 2);
        AbstractOpMode.currentOpMode().telemetry.addData("R1 - COL:", color[0]);
        AbstractOpMode.currentOpMode().telemetry.addData("R1 - COL:", color[1]);
        AbstractOpMode.currentOpMode().telemetry.addData("R1 - COL:", color[2]);
        AbstractOpMode.currentOpMode().telemetry.addData("R2 - COL:", color2[0]);
        AbstractOpMode.currentOpMode().telemetry.addData("R2 - COL:", color2[1]);
        AbstractOpMode.currentOpMode().telemetry.addData("R2 - COL:", color2[2]);
        AbstractOpMode.currentOpMode().telemetry.addData("R1 - WHITE PX:", Core.countNonZero(region1));
        AbstractOpMode.currentOpMode().telemetry.addData("R1 - WHITE/BLACK PX:", Math.round((Core.countNonZero(region1) / SIZE) * 100.0) / 100.0);
        AbstractOpMode.currentOpMode().telemetry.addData("R2 - WHITE PX:", Core.countNonZero(region2));
        AbstractOpMode.currentOpMode().telemetry.addData("R2 - WHITE/BLACK PX:", Math.round((Core.countNonZero(region2) / SIZE) * 100.0) / 100.0);

        AbstractOpMode.currentOpMode().telemetry.update();
        //Core.add(mat, mask, finalMat);
        return mask;
    }
}
