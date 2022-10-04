package org.firstinspires.ftc.teamcode.League1.Testing.Vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.opencv.utils.Converters;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class TestingContourPipeline extends OpenCvPipeline {

    Telemetry telemetry;
    Mat temp = new Mat();
    private ArrayList<Integer> xList, yList;



    public TestingContourPipeline(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, temp, Imgproc.COLOR_RGB2HSV);


        //Yellow
        Core.inRange(temp, new Scalar(25, 180, 50), new Scalar(27, 255, 255), temp);

        Imgproc.GaussianBlur(temp, temp, new Size(5, 5), 0);




        List<MatOfPoint> contours = new ArrayList<>();

        /*MatOfPoint contour1 = contours.get(0);
        MatOfPoint contour2;
*/
        //Find out last two things
        Imgproc.findContours(temp, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        Imgproc.drawContours(input, contours, -1, new Scalar(26, 230, 200));

        ArrayList<Point> pointList = new ArrayList<>();

        xList = new ArrayList<>();
        yList = new ArrayList<>();

        double pointX;
        double pointY;


        for(int i = 0; i < contours.size(); i++){
            telemetry.addData("contour" + i + ": ", contours.get(i).toArray().length);
        }

        Moments moments = Imgproc.moments(temp, true);

        Converters.Mat_to_vector_Point(contours.get(0), pointList);

        telemetry.addData("Countours: ", contours.size());
        telemetry.addData("Points: ", pointList.size());


        telemetry.update();



        return temp;




    }
}
