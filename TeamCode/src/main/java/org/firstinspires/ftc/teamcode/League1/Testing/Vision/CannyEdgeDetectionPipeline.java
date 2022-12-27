package org.firstinspires.ftc.teamcode.League1.Testing.Vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class CannyEdgeDetectionPipeline extends OpenCvPipeline {

    Telemetry telemetry;
    Mat temp = new Mat();
    Mat blurred = new Mat();
    MatOfPoint2f mMOP2f1 = new MatOfPoint2f();
    MatOfPoint2f mMOP2f2 = new MatOfPoint2f();
    private ArrayList<Integer> xList, yList;
    int cX, cY;
    Moments M;
    boolean returnInput = true;


    static final Rect MIDDLE = new Rect(
            new Point(70, 0),
            new Point(250, 176)
    );



    public CannyEdgeDetectionPipeline(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.medianBlur(input, input, 7);
        Imgproc.Canny(input, temp, 120, 220);

        Imgproc.dilate(temp, temp, new Mat());

        Core.bitwise_not(temp, temp);

        List<MatOfPoint> contours = new ArrayList<>();

        Imgproc.findContours(temp, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        for(int i = 0; i < contours.size(); i++){
            //Convert contours(i) from MatOfPoint to MatOfPoint2f
            contours.get(i).convertTo(mMOP2f1, CvType.CV_32FC2);
            //Processing on mMOP2f1 which is in type MatOfPoint2f
            double epsilon = 0.02 * Imgproc.arcLength(mMOP2f1, true);
            Imgproc.approxPolyDP(mMOP2f1, mMOP2f2, epsilon, true);
            //Convert back to MatOfPoint and put the new values back into the contours list
            mMOP2f2.convertTo(contours.get(i), CvType.CV_32S);
        }

        //Imgproc.line(temp, new Point(0, 50), new Point(320, 50), new Scalar(255));
        //Imgproc.line(temp, new Point(0, 175), new Point(320, 175), new Scalar(255));

        for(int i = 0; i < contours.size(); i++){
            Rect contourBox = new Rect();
            contourBox = Imgproc.boundingRect(contours.get(i));
            int height = contourBox.height;
            int width = contourBox.width;

            double contourArea = Imgproc.contourArea(contours.get(i));

            double percentFilled = contourArea / (width * height);

            if(contourArea > 50 && height > width * 3 /*&& percentFilled > 0.5*/&& contours.get(i).toArray().length < 8) {

                Imgproc.drawContours(input, contours, i, new Scalar(0, 255, 0));

                M = Imgproc.moments(contours.get(i));

                cX = (int)(M.m10 / M.m00);
                cY = (int)(M.m01 / M.m00);

                Imgproc.circle(input, new Point(cX, cY), 3, new Scalar(0, 255, 0));

                telemetry.addData("points", contours.get(i).toArray().length);
                Imgproc.rectangle(input, contourBox, new Scalar(0, 0, 255));
                telemetry.addData("Centroid " + i, cX + ", " + cY);
                //telemetry.update();

            }
        }

        telemetry.addData("thing", returnInput);
        telemetry.update();

        if (returnInput) {
            return input;
        } else {
            return temp;
        }
    }

    public void switchMat() {
        returnInput = !returnInput;
    }

}
