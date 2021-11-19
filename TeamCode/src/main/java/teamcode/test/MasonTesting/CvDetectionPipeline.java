package teamcode.test.MasonTesting;

import org.opencv.core.Core;
import org.opencv.core.CvType;
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

    List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
    Mat heirarchy = new Mat();

    // find contours
    public void findImgContours(Mat frame) {
        Mat labMat = new Mat(frame.rows(), frame.cols(), frame.type());
        Imgproc.cvtColor(frame, labMat, Imgproc.COLOR_RGB2Lab);
        Mat bMat = new Mat();
        Core.extractChannel(labMat, bMat, 2);

        Mat binary = new Mat(frame.rows(), frame.cols(), frame.type());
        Imgproc.threshold(bMat, binary, 100, 255, Imgproc.THRESH_BINARY_INV);

        // find contours
        Imgproc.findContours(binary, contours, heirarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        // draw contours
        Mat draw = Mat.zeros(binary.size(), CvType.CV_8UC1);
        for (int i = 0; i < contours.size(); i++) {
            Scalar color = new Scalar(255, 255, 255);
            Imgproc.drawContours(draw, contours, i, color, 2, Imgproc.LINE_8, heirarchy);
        }
    }

    @Override
    public Mat processFrame(Mat input){
        findImgContours(input);

        return input;
    }
}