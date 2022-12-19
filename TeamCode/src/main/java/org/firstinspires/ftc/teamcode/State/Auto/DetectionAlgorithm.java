package org.firstinspires.ftc.teamcode.State.Auto;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

//TODO: check if camera angle works
public class DetectionAlgorithm extends OpenCvPipeline {
    Telemetry telemetry;
    /*
   YELLOW  = Parking Left
   CYAN    = Parking Middle
   MAGENTA = Parking Right
    */
    // Mat defined/instantiated, percents (for each color) instantiated
    private Mat yelMat = new Mat(), cyaMat = new Mat(), magMat = new Mat(), changed = new Mat(), original = new Mat();
    private double yelPercent, cyaPercent, magPercent;

    // top left point of submat (original 320, 176)
    public static final Point BOX_BOTTOMLEFT = new Point(175,150);

    // width and height of submat
    public static int BOX_WIDTH = 30;
    public static int BOX_HEIGHT = -35;

    enum ParkingPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    // Running variable storing the parking position
    private volatile ParkingPosition position = ParkingPosition.LEFT;

    // submat to center cone
    Point box_top_left = new Point(
            BOX_BOTTOMLEFT.x,
            BOX_BOTTOMLEFT.y);
    Point box_bottom_right = new Point(
            BOX_BOTTOMLEFT.x + BOX_WIDTH,
            BOX_BOTTOMLEFT.y + BOX_HEIGHT);

    // Lower and upper boundaries for colors
    private static final Scalar
            lower_yellow_bounds  = new Scalar(218, 218, 0, 255),
            upper_yellow_bounds  = new Scalar(255, 255, 135, 255),
            lower_cyan_bounds    = new Scalar(0, 176, 176, 255),
            upper_cyan_bounds    = new Scalar(150, 255, 255, 255),
            lower_magenta_bounds = new Scalar(170, 0, 170, 255),
            upper_magenta_bounds = new Scalar(255, 95, 255, 255);

    // Color definitions
    private final Scalar
            YELLOW  = new Scalar(255, 255, 0),
            CYAN    = new Scalar(0, 255, 255),
            MAGENTA = new Scalar(255, 0, 255);

    public DetectionAlgorithm(Telemetry telemetry){
        this.telemetry = telemetry;
    }


    @Override
    public Mat processFrame(Mat input) {
        Mat magCr, befChange = new Mat();

        input.copyTo(original);

        if(original.empty()) {
            return input;
        }
        // cyan magenta yellow

        /* colors (scalars):
            magenta -> new Scalar(255, 0, 255)
            yellow -> new Scalar(255, 255, 0)
            cyan -> new Scalar(0, 255, 255)
         */
        changed = original.submat(new Rect(box_top_left, box_bottom_right));

        //Core.extractChannel(changed, changed, 1);
        //Imgproc.cvtColor(original, changed, Imgproc.COLOR_RGB2YCrCb);
        Imgproc.GaussianBlur(changed, changed, new Size(5,5), 0);
        Imgproc.erode(changed, changed, new Mat(), new Point(-1, -1), 2);
        Imgproc.dilate(changed, changed, new Mat(), new Point(-1, -1), 2);
        befChange = changed.submat(new Rect(new Point(11, 11), new Point(12, 12)));
//        // magenta (ycrcb worked better for magenta)
//        Imgproc.cvtColor(changed, magCr, Imgproc.COLOR_RGB2YCrCb);
//        Core.extractChannel(magCr, magMat, 1);
//        Core.inRange(magMat, new Scalar(190), new Scalar(240), magMat);



        // container

        //Imgproc.rectangle(original, new Rect(box_top_left, box_bottom_right), new Scalar(255, 153, 204));

            /* submatrices
            Mat pixel_section = original.submat(rowStart, rowEnd, colStart, colEnd)l

             */

        // yellow
        Core.inRange(changed, lower_yellow_bounds, upper_yellow_bounds, yelMat);
        // cyan
        Core.inRange(changed, lower_cyan_bounds, upper_cyan_bounds, cyaMat);
        // magenta
        Core.inRange(changed, lower_magenta_bounds, upper_magenta_bounds, magMat);



        // percent "abundance" for each color
        yelPercent = Core.countNonZero(yelMat);
        cyaPercent = Core.countNonZero(cyaMat);
        magPercent = Core.countNonZero(magMat);
        telemetry.addData("yelPercent", yelPercent);
        telemetry.addData("cyaPercent", cyaPercent);
        telemetry.addData("magPercent", magPercent);
        telemetry.addData("cyanColor", Core.sumElems(befChange).val[2]);
        telemetry.addData("magColor", Core.sumElems(befChange).val[1]);
        telemetry.addData("yelColor", Core.sumElems(befChange).val[0]);
        telemetry.update();

        // decides parking position, highlights margin according to greatest abundance color
        if (yelPercent > cyaPercent) {
            if (yelPercent > magPercent) {
                // yellow greatest, position left
                position = ParkingPosition.LEFT;
//                telemetry.addData("park position", position);
                Imgproc.rectangle(original, new Rect(box_top_left, box_bottom_right), YELLOW, 2);
            } else {
                // magenta greatest, position right
                position = ParkingPosition.RIGHT;
//                telemetry.addData("park position", position);
                Imgproc.rectangle(original, new Rect(box_top_left, box_bottom_right), MAGENTA, 2);
            }
        } else if(cyaPercent > magPercent) {
            // cyan greatest, position center
            position = ParkingPosition.CENTER;
//            telemetry.addData("park position", position);
            Imgproc.rectangle(original, new Rect(box_top_left, box_bottom_right), CYAN, 2);
        } else {

            // magenta greatest, position right
            position = ParkingPosition.RIGHT;
//            telemetry.addData("park position", position);
            Imgproc.rectangle(original, new Rect(box_top_left, box_bottom_right), MAGENTA, 2);

        }
        telemetry.update();

        // Memory cleanup
        //changed.release();
        //original.release();
        yelMat.release();
        cyaMat.release();
        magMat.release();

        return original;
    }

    // Returns an enum being the current position where the robot will park
    public ParkingPosition getPosition() {
        return position;
    }
}




