package org.firstinspires.ftc.teamcode.Auto;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

//TODO: check if camera angle works
public class DetectionAlgorithmTest extends OpenCvPipeline {


    Mat original;
    private Mat yelMat = new Mat(), cyaMat = new Mat(), magMat = new Mat(), changed = new Mat();

    // top left point of submat
    public static final Point BOX_TOPLEFT = new Point(100,176);

    // width and height of submat
    public static int BOX_WIDTH = 150;
    public static int BOX_HEIGHT = -88;

    enum ParkingPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    // submat to center cone
    Point box_top_left = new Point(
            BOX_TOPLEFT.x,
            BOX_TOPLEFT.y);
    Point box_bottom_right = new Point(
            BOX_TOPLEFT.x + BOX_WIDTH,
            BOX_TOPLEFT.y + BOX_HEIGHT);

    // Lower and upper boundaries for colors
    private static final Scalar
            lower_yellow_bounds  = new Scalar(200, 200, 0, 255),
            upper_yellow_bounds  = new Scalar(255, 255, 130, 255),
            lower_cyan_bounds    = new Scalar(0, 200, 200, 255),
            upper_cyan_bounds    = new Scalar(150, 255, 255, 255),
            lower_magenta_bounds = new Scalar(170, 0, 170, 255),
            upper_magenta_bounds = new Scalar(255, 60, 255, 255);

    // Running variable storing the parking position
    private volatile ParkingPosition position = ParkingPosition.LEFT;

    @Override
    public Mat processFrame(Mat input) {
        original = new Mat();

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
        Imgproc.GaussianBlur(changed, changed, new Size(3,3), 0);
        Imgproc.erode(changed, changed, new Mat(), new Point(-1, -1), 3);
        Imgproc.dilate(changed, changed, new Mat(), new Point(-1, -1), 3);


            /* submatrices
            Mat pixel_section = original.submat(rowStart, rowEnd, colStart, colEnd)l

             */



        // magenta 255, 0, 255
        //Core.inRange(changed, new Scalar(240, 0 ,240), new Scalar(255, 0, 255), changed);

        Imgproc.rectangle(original, new Rect(box_top_left, box_bottom_right), new Scalar(255, 0, 255));
        return changed;
    }

    // Returns an enum being the current position where the robot will park
    public ParkingPosition getPosition() {
        return position;
    }
}




