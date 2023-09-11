package org.firstinspires.ftc.teamcode.Competition.DebuggingOrTuning.Vision.Old.Pipelines;

////import com.acmerobotics.dashboard.config.Config;

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
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

////@Config
public class ContourPipeline extends OpenCvPipeline {

    public static int H1 = 23;
    public static int S1 = 50;
    public static int V1 = 50;
    public static int H2 = 33;
    public static int S2 = 200;
    public static int V2 = 200;

    Telemetry telemetry;
    Mat temp = new Mat();
    private ArrayList<Integer> xList, yList, contourLengths;
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

        xList = new ArrayList<>();
        yList = new ArrayList<>();
        contourLengths = new ArrayList<>();
        this.telemetry = telemetry;
    }

    @Override
    public Mat processFrame(Mat input) {


        Imgproc.cvtColor(input, temp, Imgproc.COLOR_RGB2HSV);


        //Yellow
        Core.inRange(temp, new Scalar(H1, S1, V1), new Scalar(H2, S2, V2), temp);

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
                contourLengths.add(contours.get(i).toArray().length);
                xList.add(cX);
                yList.add(cY);

                telemetry.addData("Centroid " + i, cX + ", " + cY);
                telemetry.update();

            }
        }

        for(int i = 0; i < xList.size() && i < contourLengths.size() && i < yList.size(); i++) {
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



        telemetry.addData("Contours", contourLengths.size());
        telemetry.addData("X", xList.size());
        telemetry.addData("Y", yList.size());
        telemetry.update();
        contourLengths.clear();
        xList.clear();
        yList.clear();
        maxLength = 0;


        return input;




    }

    public int getPolePosition() {
        return longestContourX;
    }

}
