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
    Mat blurred = new Mat();
    private ArrayList<Integer> xList, yList;
    int cX, cY;
    Moments M;


    static final Rect MIDDLE = new Rect(
            new Point(70, 0),
            new Point(250, 176)
    );



    public TestingContourPipeline(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.medianBlur(input, blurred, 13);

        Imgproc.cvtColor(input, temp, Imgproc.COLOR_RGB2HSV);


        //Yellow
        Core.inRange(temp, new Scalar(25, 180, 50), new Scalar(27, 255, 255), temp);

        Imgproc.GaussianBlur(temp, temp, new Size(5, 5), 0);

        Mat thing = temp.submat(MIDDLE);
        Imgproc.rectangle(input, MIDDLE, new Scalar(0, 0, 0), 3);

        //Imgproc.GaussianBlur(thing, thing, new Size(101, 101), 0);

        List<MatOfPoint> contours = new ArrayList<>();

        /*MatOfPoint contour1 = contours.get(0);
        MatOfPoint contour2;
*/
        //Find out last two things
        Imgproc.findContours(temp, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        //Imgproc.drawContours(input, contours, -1, new Scalar(26, 230, 200));

        for(int i = 0; i < contours.size(); i++){
            if(contours.get(i).toArray().length > 6){
                Imgproc.drawContours(input, contours, i, new Scalar(230, 191, 254));

                M = Imgproc.moments(contours.get(i));

                cX = (int)(M.m10 / M.m00);
                cY = (int)(M.m01 / M.m00);

                Imgproc.circle(input, new Point(cX, cY), 3, new Scalar(0, 255, 0));

                telemetry.addData("Centroid " + i, cX + ", " + cY);
                telemetry.update();

                /*ArrayList<Point> pointList = new ArrayList<>();
                Converters.Mat_to_vector_Point(contours.get(i), pointList);
                for(int j = 0; j < pointList.size(); j++){
                    pointList.get(j).x += 90;
                }
                contours.get(i).

                 */
            }
        }
        /*
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
*/


        return blurred;




    }
}
