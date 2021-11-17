package teamcode.test.MasonTesting;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

class CvDetectionPipeline extends OpenCvPipeline {

    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar WHITE = new Scalar(255, 255, 255);

    static final Point CREGION_TOPLEFT_ANCHOR_POINT = new Point(150, 110);
    static final int REGION_WIDTH = 20;
    static final int REGION_HEIGHT = 20;

    Point cRegion_pointA = new Point(
            CREGION_TOPLEFT_ANCHOR_POINT.x,
            CREGION_TOPLEFT_ANCHOR_POINT.y);
    Point cRegion_pointB = new Point(
            CREGION_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            CREGION_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
    Mat heirarchy = new Mat();

    // TODO - research docs for following statements, and tweak values in regards to HSV and binary vals
    // convert img to binary
    public Mat srcToHSVBinary(Mat input) {
        Mat hsvMat = new Mat(input.rows(), input.cols(), input.type());
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
        Mat binary = new Mat(input.rows(), input.cols(), input.type(), new Scalar(0));
        Imgproc.threshold(hsvMat, binary, 100, 255, Imgproc.THRESH_BINARY_INV);

        return binary;
    }

    // find contours
    public void findImgContours(Mat binaryMat) {
        Imgproc.findContours(binaryMat, contours, heirarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
    }

    @Override
    public Mat processFrame(Mat input){
        srcToHSVBinary(input);
        findImgContours(input);

        return input;
    }

    public String imgToString() {
        return contours.toString();
    }
}