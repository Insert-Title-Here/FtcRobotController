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
    static final Scalar RED = new Scalar(255, 0, 0);

    static final int REGION_WIDTH = 65;
    static final int REGION_HEIGHT = 30;

    static final double SIZE = REGION_WIDTH * REGION_HEIGHT;

    static Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(108, 145);
    static Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(251, 145);

    static double deltaPercent = 0.0;
    static double regionOnePercent = 0.0;
    static double regionTwoPercent = 0.0;

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
            REGION1_TOPLEFT_ANCHOR_POINT = new Point(135, 145);
            REGION2_TOPLEFT_ANCHOR_POINT = new Point(255, 140);
            Core.inRange(filtered, new Scalar(0, 20, 40), new Scalar(75, 110, 255), mask);
        } else {
            REGION1_TOPLEFT_ANCHOR_POINT = new Point(0, 155);
            REGION2_TOPLEFT_ANCHOR_POINT = new Point(134, 150);
            //Core.inRange(filtered, new Scalar(230, 115, 70), new Scalar(255, 148, 97), mask);
            Core.inRange(filtered, new Scalar(99, 35, 0), new Scalar(255, 210, 162), mask);
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
                RED,
                2
        );
        Imgproc.rectangle(
                mask,
                region2_pointA,
                region2_pointB,
                RED,
                2
        );

        regionOnePercent = Math.round((Core.countNonZero(region1) / SIZE) * 100.0) / 100.0;
        regionTwoPercent = Math.round((Core.countNonZero(region2) / SIZE) * 100.0) / 100.0;

        deltaPercent = Math.abs(regionOnePercent - regionTwoPercent);
        if(side == Side.RED) {
            if(deltaPercent < 0.15){
                position = BarcodePosition.RIGHT;
            }else if(regionOnePercent < regionTwoPercent){
                position = BarcodePosition.CENTER;
            }else if(regionTwoPercent < regionOnePercent){
                position = BarcodePosition.LEFT;
            }else{
                position = BarcodePosition.RIGHT;
            }
        }else{
            if(deltaPercent < 0.1){
                position = BarcodePosition.LEFT;
            }else if(regionOnePercent < regionTwoPercent){
                position = BarcodePosition.CENTER;
            }else if(regionTwoPercent < regionOnePercent){
                position = BarcodePosition.RIGHT;
            }else{
                position = BarcodePosition.RIGHT;
            }
        }


        double[] color = mat.get(170, 152);
        Imgproc.circle(mask, new Point(152, 170), 5, WHITE, 2);
        double[] color2 = mat.get(170, 166);
        //Imgproc.circle(mask, new Point(166, 170), 5, WHITE, 2);
        //Imgproc.circle(mat, new Point(160, 120), 5, WHITE, 2);
        AbstractOpMode.currentOpMode().telemetry.addData("R1 - COL:", color[0]);
        AbstractOpMode.currentOpMode().telemetry.addData("R1 - COL:", color[1]);
        AbstractOpMode.currentOpMode().telemetry.addData("R1 - COL:", color[2]);
//        AbstractOpMode.currentOpMode().telemetry.addData("R2 - COL:", color2[0]);
//        AbstractOpMode.currentOpMode().telemetry.addData("R2 - COL:", color2[1]);
//        AbstractOpMode.currentOpMode().telemetry.addData("R2 - COL:", color2[2]);
//        AbstractOpMode.currentOpMode().telemetry.addData("R1 - WHITE PX:", Core.countNonZero(region1));
//        AbstractOpMode.currentOpMode().telemetry.addData("R1 - WHITE/BLACK PX:", Math.round((Core.countNonZero(region1) / SIZE) * 100.0) / 100.0);
//        AbstractOpMode.currentOpMode().telemetry.addData("R2 - WHITE PX:", Core.countNonZero(region2));
//        AbstractOpMode.currentOpMode().telemetry.addData("R2 - WHITE/BLACK PX:", Math.round((Core.countNonZero(region2) / SIZE) * 100.0) / 100.0);
//        AbstractOpMode.currentOpMode().telemetry.addData("POS", getPos());
//        AbstractOpMode.currentOpMode().telemetry.addData("SIDE", getSide());
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

    public String getVal() {
        return "delta %: " + deltaPercent + "\nregion 1%: " + regionOnePercent + "\nregion 2%: " + regionTwoPercent;
    }
}
