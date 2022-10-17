package org.firstinspires.ftc.teamcode.League1.Autonomous.Vision;

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
import java.util.Collections;
import java.util.List;

public class ContourPipeline extends OpenCvPipeline {

    Telemetry telemetry;
    Mat temp = new Mat();
    private ArrayList<Integer> xList, yList, contourLengths = new ArrayList<Integer>();
    int cX, cY;
    int maxLength = 0;
    int maxLengthIndex = 0;
    int longestContourX = 0;
    Moments M;


    static final Rect MIDDLE = new Rect(
            new Point(70, 0),
            new Point(250, 176)
    );



    public ContourPipeline(Telemetry telemetry){
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {


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

                Imgproc.circle(input, new Point(cX, cY), 3, new Scalar(0, 0, 255));

                //Problem with one of the next 3 lines probably
                contourLengths.add(i, contours.get(i).toArray().length);
                xList.add(i, cX);
                yList.add(i, cY);

                telemetry.addData("Centroid " + i, cX + ", " + cY);
                telemetry.update();

            }
        }


        for(int i = 0; i < contourLengths.size(); i++) {
            if(contourLengths.get(i) > maxLength) {
                maxLength = contourLengths.get(i);
                maxLengthIndex = i;
            }
        }

        if(contourLengths.size() > 0) {
            longestContourX = xList.get(maxLengthIndex);
            Imgproc.circle(input, new Point(xList.get(maxLengthIndex), yList.get(maxLengthIndex)), 3, new Scalar(0, 255, 0));
        }

        telemetry.addData("Contour X Pos", longestContourX);

        contourLengths.clear();
        xList.clear();
        yList.clear();


        return input;




    }

    public int getPolePosition() {
        return longestContourX;
    }

}
