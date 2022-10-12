package org.firstinspires.ftc.teamcode.Auto;


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
public class DetectionAlgorithmTest2 extends OpenCvPipeline {
    Telemetry telemetry;
    /*
   YELLOW  = Parking Left
   CYAN    = Parking Middle
   MAGENTA = Parking Right
    */
    // Mat defined/instantiated, percents (for each color) instantiated
    private Mat yelMat = new Mat(), cyaMat = new Mat(), magMat = new Mat(), changed = new Mat(), original = new Mat();
    private double yelPercent, cyaPercent, magPercent;

    // top left point of submat
    public static final Point BOX_TOPLEFT = new Point(100,176);

    // width and height of submat
    public static int BOX_WIDTH = 100;
    public static int BOX_HEIGHT = -88;

    enum ParkingPosition {
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

    // Lower and upper boundaries for colors -> HSV
    private static final Scalar
            lower_yellow_bounds  = new Scalar(
                    convertToHSV(200, 200, 0)[0],
                    convertToHSV(200, 200, 0)[1],
                    convertToHSV(200, 200, 0)[2]),
            upper_yellow_bounds  = new Scalar(
                    convertToHSV(255, 255, 130)[0],
                    convertToHSV(255, 255, 130)[1],
                    convertToHSV(255, 255, 130)[2]),
            lower_cyan_bounds    = new Scalar(
                    convertToHSV(0, 200, 200)[0],
                    convertToHSV(0, 200, 200)[1],
                    convertToHSV(0, 200, 200)[2]),
            upper_cyan_bounds    = new Scalar(
                    convertToHSV(150, 255, 255)[0],
                    convertToHSV(150, 255, 255)[1],
                    convertToHSV(150, 255, 255)[2]),
            lower_magenta_bounds = new Scalar(
                    convertToHSV(170, 0, 170)[0],
                    convertToHSV(170, 0, 170)[1],
                    convertToHSV(170, 0, 170)[2]),
            upper_magenta_bounds = new Scalar(
                    convertToHSV(255, 60, 255)[0],
                    convertToHSV(255, 60, 255)[1],
                    convertToHSV(255, 60, 255)[2]);

    // Color definitions -> RGB
    private final Scalar
            YELLOW  = new Scalar(255, 255, 0),
            CYAN    = new Scalar(0, 255, 255),
            MAGENTA = new Scalar(255, 0, 255);

    public DetectionAlgorithmTest2(Telemetry telemetry){
        this.telemetry = telemetry;
    }


    @Override
    public Mat processFrame(Mat input) {
        Mat befChange = new Mat();

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
        changed.copyTo(befChange);

        //Core.extractChannel(changed, changed, 1);

        Imgproc.GaussianBlur(changed, changed, new Size(5,5), 0);
        Imgproc.erode(changed, changed, new Mat(), new Point(-1, -1), 2);
        Imgproc.dilate(changed, changed, new Mat(), new Point(-1, -1), 2);
        Imgproc.cvtColor(changed, changed, Imgproc.COLOR_RGB2HSV);

        //h -> Hue, S -> Saturation, V -> Value
//        // Apply Morphology
//        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
//        Imgproc.morphologyEx(changed, changed, Imgproc.MORPH_CLOSE, kernel);
//


            /* submatrices
            Mat pixel_section = original.submat(rowStart, rowEnd, colStart, colEnd)l

             */

        // https://sistenix.com/rgb2ycbcr.html -> convert between rgb and ycbcr
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
        telemetry.update();

        // decides parking position, highlights margin according to greatest abundance color
        if (yelPercent > cyaPercent) {
            if (yelPercent > magPercent) {
                // yellow greatest, position left
                position = ParkingPosition.LEFT;
                telemetry.addData("park position", position);
                Imgproc.rectangle(original, new Rect(box_top_left, box_bottom_right), YELLOW, 2);
            } else {
                // magenta greatest, position right
                position = ParkingPosition.RIGHT;
                telemetry.addData("park position", position);
                Imgproc.rectangle(original, new Rect(box_top_left, box_bottom_right), MAGENTA, 2);
            }
        } else if(cyaPercent > magPercent) {
            // cyan greatest, position center
            position = ParkingPosition.CENTER;
            telemetry.addData("park position", position);
            Imgproc.rectangle(original, new Rect(box_top_left, box_bottom_right), CYAN, 2);
        } else {

            // magenta greatest, position right
            position = ParkingPosition.RIGHT;
            telemetry.addData("park position", position);
            Imgproc.rectangle(original, new Rect(box_top_left, box_bottom_right), MAGENTA, 2);

        }
        telemetry.update();

        // Memory cleanup
        //changed.release();
        //original.release();
        //yelMat.release();
        cyaMat.release();
        magMat.release();

        return yelMat;
    }


    public static Double[] convertToHSV(double r, double g, double b) {
        // https://www.geeksforgeeks.org/program-change-rgb-color-model-hsv-color-model/ with my own customizations

        // R, G, B values are divided by 255
        // to change the range from 0..255 to 0..1
        r = r / 255.0;
        g = g / 255.0;
        b = b / 255.0;

        // h, s, v = hue, saturation, value
        double cmax = Math.max(r, Math.max(g, b)); // maximum of r, g, b
        double cmin = Math.min(r, Math.min(g, b)); // minimum of r, g, b
        double diff = cmax - cmin; // diff of cmax and cmin.
        double h = -1, s = -1;

        // if cmax and cmax are equal then h = 0
        if (cmax == cmin)
            h = 0;

            // if cmax equal r then compute h
        else if (cmax == r)
            h = ((60 * ((g - b) / diff) + 360) % 360 ) ;

            // if cmax equal g then compute h
        else if (cmax == g)
            h = ((60 * ((b - r) / diff) + 120) % 360 ) ;

            // if cmax equal b then compute h
        else if (cmax == b)
            h = ((60 * ((r - g) / diff) + 240) % 360 ) ;

        // if cmax equal zero
        if (cmax == 0)
            s = 0;
        else
            s = ((diff / cmax) * 100 );

        // compute v
        double v = cmax * 100;
        Double[] hsv = {h, s, v};
        return hsv;

    }




    // Returns an enum being the current position where the robot will park
    public ParkingPosition getPosition() {
        return position;
    }
}




