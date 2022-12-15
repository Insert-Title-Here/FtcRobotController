package org.firstinspires.ftc.teamcode.Auto;


import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config //TODO: check if camera angle works
public class DetectionAlgorithmLeft extends OpenCvPipeline {
    Telemetry telemetry;
    /*
   YELLOW  = Parking Left
   CYAN    = Parking Middle
   MAGENTA = Parking Right
    */
    // Mat defined/instantiated, percents (for each color) instantiated
    private Mat yelMat = new Mat(), cyaMat = new Mat(), magMat = new Mat(), changed = new Mat(), original = new Mat();
    private double yelPercent, cyaPercent, magPercent;

    public static int x = 200; // 224
    public static int y = 89; //95

    // top left point of submat (original 320, 176)
    public static Point BOX_TOPLEFT = new Point(x,y); // 175, 150 ... 175, 114

    // width and height of submat
    public static int BOX_WIDTH = 23;
    public static int BOX_HEIGHT = -38;

    public enum ParkingPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    // Running variable storing the parking position
    private volatile ParkingPosition position = ParkingPosition.LEFT;

    // submat to center cone
    Point box_top_left = new Point(
            BOX_TOPLEFT.x,
            BOX_TOPLEFT.y);
    Point box_bottom_right = new Point(
            BOX_TOPLEFT.x + BOX_WIDTH,
            BOX_TOPLEFT.y + BOX_HEIGHT);

    // Lower and upper boundaries for colors -> YCrCb
    private static final Scalar
            lower_yellow_bounds  = new Scalar(convertToY(200, 200, 0)),
            upper_yellow_bounds  = new Scalar(convertToY(255, 255, 130)),
            lower_cyan_bounds    = new Scalar(convertToCb(0, 200, 200)),
            upper_cyan_bounds    = new Scalar(convertToCb(150, 255, 255)),
            lower_magenta_bounds = new Scalar(convertToCr(170, 0, 170)),
            upper_magenta_bounds = new Scalar(convertToCr(255, 60, 255));

    // Color definitions -> RGB
    private final Scalar
            YELLOW  = new Scalar(255, 255, 0),
            CYAN    = new Scalar(0, 255, 255),
            MAGENTA = new Scalar(255, 0, 255);

    public DetectionAlgorithmLeft(Telemetry telemetry){
        this.telemetry = telemetry;
    }


    @Override
    public Mat processFrame(Mat input) {

//        BOX_TOPLEFT = new Point(x,y); // 175, 150 ... 175, 114
//
//        // width and height of submat
//        BOX_WIDTH = 23;
//        BOX_HEIGHT = -38;
//
//        Point box_top_left = new Point(
//                BOX_TOPLEFT.x,
//                BOX_TOPLEFT.y);
//        Point box_bottom_right = new Point(
//                BOX_TOPLEFT.x + BOX_WIDTH,
//                BOX_TOPLEFT.y + BOX_HEIGHT);
        Mat befChange = new Mat();

        input.copyTo(original);

        if(original.empty()) {
            return input;
        }

        changed = original.submat(new Rect(box_top_left, box_bottom_right));

        //Core.extractChannel(changed, changed, 1);

        Imgproc.GaussianBlur(changed, changed, new Size(5,5), 0);
        Imgproc.erode(changed, changed, new Mat(), new Point(-1, -1), 2);
        Imgproc.dilate(changed, changed, new Mat(), new Point(-1, -1), 2);
        Imgproc.cvtColor(changed, changed, Imgproc.COLOR_RGB2YCrCb);
        befChange = changed.submat(new Rect(new Point(11, 11), new Point(12, 12)));
        //Y -> brightness, Cr -> red - brightness, Cb -> blue - brightness
        Core.extractChannel(changed, yelMat, 0);
        Core.extractChannel(changed, cyaMat, 2);
        Core.extractChannel(changed, magMat, 1);
//        // Apply Morphology
//        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
//        Imgproc.morphologyEx(changed, changed, Imgproc.MORPH_CLOSE, kernel);
//


            /* submatrices
            Mat pixel_section = original.submat(rowStart, rowEnd, colStart, colEnd)l

             */

        // https://sistenix.com/rgb2ycbcr.html -> convert between rgb and ycbcr
        // yellow 190
//        Core.inRange(yelMat, lower_yellow_bounds, upper_yellow_bounds, yelMat);
        Core.inRange(yelMat, new Scalar(160), new Scalar(205), yelMat);
        // cyan 169
//        Core.inRange(cyaMat, lower_cyan_bounds, upper_cyan_bounds, cyaMat);
        Core.inRange(cyaMat, new Scalar(160), new Scalar(184), cyaMat);
        // magenta 186
//        Core.inRange(magMat, lower_magenta_bounds, upper_magenta_bounds, magMat);
        Core.inRange(magMat, new Scalar(161), new Scalar(195), magMat);

//        befChange.convertTo(befChange, CvType.CV_64FC3);
//        byte buff[] = new byte[ (int) (befChange.total() * befChange.channels())];
//        byte buff2[] = new byte[3];
        // percent "abundance" for each color
        yelPercent = Core.countNonZero(yelMat);
        cyaPercent = Core.countNonZero(cyaMat);
        magPercent = Core.countNonZero(magMat);
        telemetry.addData("yelPercent", yelPercent);
        telemetry.addData("cyaPercent", cyaPercent);
        telemetry.addData("magPercent", magPercent);
//        telemetry.addData("cyanColor", changed.get(0, 0,buff2));
        telemetry.addData("cyanColor", Core.sumElems(befChange).val[2]);
        telemetry.addData("magColor", Core.sumElems(befChange).val[1]);
        telemetry.addData("yelColor", Core.sumElems(befChange).val[0]);
        telemetry.update();

        // decides parking position, highlights margin according to greatest abundance color
        if (yelPercent > cyaPercent) {
            if (yelPercent > magPercent) {
                // yellow greatest, position left
                position = ParkingPosition.LEFT;
                //telemetry.addData("park position", position);
                Imgproc.rectangle(original, new Rect(box_top_left, box_bottom_right), YELLOW, 2);
            } else {
                // magenta greatest, position right
                position = ParkingPosition.RIGHT;
                //telemetry.addData("park position", position);
                Imgproc.rectangle(original, new Rect(box_top_left, box_bottom_right), MAGENTA, 2);
            }
        } else if(cyaPercent > magPercent) {
            // cyan greatest, position center
            position = ParkingPosition.CENTER;
            //telemetry.addData("park position", position);
            Imgproc.rectangle(original, new Rect(box_top_left, box_bottom_right), CYAN, 2);
        } else {

            // magenta greatest, position right
            position = ParkingPosition.RIGHT;
            //telemetry.addData("park position", position);
            Imgproc.rectangle(original, new Rect(box_top_left, box_bottom_right), MAGENTA, 2);

        }
        telemetry.update();

        // Memory cleanup
        changed.release();
        //original.release();
        yelMat.release();
        cyaMat.release();
        magMat.release();

        return original;
    }
    public static double convertToY(int r, int g, int b) {
        return 16 + (65.738 * r + 129.057 * g + 25.064 * b) / 256;
    }

    public static double convertToCb(int r, int g, int b) {
        return 128 - (37.945 * r - 74.494 * g + 112.439 * b) / 256;
    }

    public static double convertToCr(int r, int g, int b) {
        return 128 + (112.439 * r - 94.154 * g - 18.285 * b) / 256;
    }

    // Returns an enum being the current position where the robot will park
    public ParkingPosition getPosition() {
        return position;
    }
}




