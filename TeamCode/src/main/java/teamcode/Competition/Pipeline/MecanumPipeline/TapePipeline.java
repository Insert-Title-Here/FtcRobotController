package teamcode.Competition.Pipeline.MecanumPipeline;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import teamcode.common.AbstractOpMode;

public class TapePipeline extends OpenCvPipeline {

    public enum BarcodePosition {
        LEFT,
        CENTER,
        RIGHT
    }

    public enum Side {
        RED,
        BLUE
    }

    volatile Side side = Side.RED;
    volatile BarcodePosition position = BarcodePosition.RIGHT;

    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);
    static final Scalar WHITE = new Scalar(255, 255, 255);

    static final int REGION_WIDTH = 65;
    static final int REGION_HEIGHT = 30;

    static final double SIZE = REGION_WIDTH * REGION_HEIGHT;

    static Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(108, 145);
    static Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(251, 145);

    Mat filtered = new Mat();
    Mat mask = new Mat();
    Mat region1 = new Mat();
    Mat region2 = new Mat();
    Mat finalMat = new Mat();

    @Override
    public Mat processFrame(Mat mat) {



        Imgproc.cvtColor(mat, filtered, Imgproc.COLOR_RGB2BGR);
        //Core.inRange(filtered, new Scalar(50, 90, 235), new Scalar(80, 117, 255), mask);
        if (side == Side.RED) {
            REGION1_TOPLEFT_ANCHOR_POINT = new Point(130, 150);
            REGION2_TOPLEFT_ANCHOR_POINT = new Point(255, 150);
            Core.inRange(filtered, new Scalar(0, 60, 40), new Scalar(40, 110, 255), mask);
        } else {
            REGION1_TOPLEFT_ANCHOR_POINT = new Point(0, 155);
            REGION2_TOPLEFT_ANCHOR_POINT = new Point(134, 150);
            //Core.inRange(filtered, new Scalar(230, 115, 70), new Scalar(255, 148, 97), mask);
            Core.inRange(filtered, new Scalar(99, 35, 0), new Scalar(255, 192, 162), mask);
        }

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

        double regionOnePercent = Math.round((Core.countNonZero(region1) / SIZE) * 100.0) / 100.0;
        double regionTwoPercent = Math.round((Core.countNonZero(region2) / SIZE) * 100.0) / 100.0;

        if (side == Side.RED) {
            if (!(regionOnePercent > 0.31 && regionOnePercent < 0.51)) {
                position = BarcodePosition.LEFT;
            } else if (!(regionTwoPercent > 0.38 && regionTwoPercent < 0.52)) {
                position = BarcodePosition.CENTER;
            } else {
                position = BarcodePosition.RIGHT;
            }
        } else {
            if (!(regionOnePercent > 0.31 && regionOnePercent < 0.47)) {
                position = BarcodePosition.CENTER;
            } else if (!(regionTwoPercent > 0.31 && regionTwoPercent < 0.47)) {
                position = BarcodePosition.RIGHT;
            } else {
                position = BarcodePosition.LEFT;
            }
        }


        double[] color = mat.get(170, 28);
        //Imgproc.circle(mask, new Point(28, 170), 5, WHITE, 2);
        double[] color2 = mat.get(170, 166);
        //Imgproc.circle(mask, new Point(166, 170), 5, WHITE, 2);
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
        AbstractOpMode.currentOpMode().telemetry.addData("POS", getPos());
        AbstractOpMode.currentOpMode().telemetry.addData("SIDE", getSide());
        AbstractOpMode.currentOpMode().telemetry.update();
        //Core.add(mat, mask, finalMat);
        return mask;
    }

    public Side getSide() {
        return this.side;
    }

    public void setSide(Side side) {
        this.side = side;
    }

    public BarcodePosition getPos() {
        return position;
    }
}
