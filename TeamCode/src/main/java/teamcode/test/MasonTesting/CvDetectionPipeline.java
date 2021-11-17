package teamcode.test.MasonTesting;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

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

    Mat HSV = new Mat();
    Mat cRegion = new Mat();
    Mat H, S, V;
    Mat H2, S2, V2;
    int h, s, v;

    // TODO - research docs for following statements, and tweak values in regards to HSV and binary vals
    // convert img to binary
    public Mat srcToHSVBinary(Mat input) {
        Mat hsvMat = new Mat(input.rows(), input.cols(), input.type());
        Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
        Mat binary = new Mat(input.rows(), input.cols(), input.type(), new Scalar(0));
        Imgproc.threshold(hsvMat, binary, 100, 255, Imgproc.THRESH_BINARY_INV);

        return null;
    }

    // find contours

    @Override
    public Mat processFrame(Mat input) {


        H2 = H.submat(new Rect(cRegion_pointA, cRegion_pointB));
        S2 = S.submat(new Rect(cRegion_pointA, cRegion_pointB));
        V2 = V.submat(new Rect(cRegion_pointA, cRegion_pointB));

        h = (int) Core.mean(H2).val[0];
        s = (int) Core.mean(S2).val[0];
        v = (int) Core.mean(V2).val[0];

        Imgproc.rectangle(
                input,
                cRegion_pointA,
                cRegion_pointB,
                BLUE,
                2
        );

        return input;
    }

    public int hVal() {
        return h;
    }

    public int sVal() {
        return s;
    }

    public int vVal() {
        return v;
    }
}