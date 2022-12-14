package org.firstinspires.ftc.teamcode.League1.Autonomous.Vision;

import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.League1.Subsystems.MecDrive;
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

@Config
public class KevinGodPipelineV2 extends OpenCvPipeline {

    // Configuration variables for isolating pole color
    public static int H1 = 0;
    public static int S1 = 100; //was 100
    public static int V1 = 80; //was 150
    public static int H2 = 50;
    public static int S2 = 255;
    public static int V2 = 255;

    //Blue cone color
    public static int H3 = 105;
    public static int S3 = 125;
    public static int V3 = 90;
    public static int H4 = 130;
    public static int S4 = 255;
    public static int V4 = 255;

    //Red cone color
    public static int H5 = 170;
    public static int S5 = 120;
    public static int V5 = 150;
    public static int H6 = 180;
    public static int S6 = 240;
    public static int V6 = 255;

    // Config variables for signal pipeline
    public static int YUpper = 200;
    public static int YLower = 100;
    public static int CrUpper = 210;
    public static int CrLower = 160;
    public static int CbUpper = 200;
    public static int CbLower = 170;

    // Config variables for bounding box
    public static int topLeftXRightRed = 180 ;
    public static int topLeftYRightRed = 50;
    public static int boxWidthRightRed = 20;
    public static int boxHeightRightRed = 40;

    public static int topLeftXLeftRed = 140;
    public static int topLeftYLeftRed = 20;
    public static int boxWidthLeftRed = 20;
    public static int boxHeightLeftRed = 40;

    public static int topLeftXRightBlue = 180;
    public static int topLeftYRightBlue = 60;
    public static int boxWidthRightBlue = 20;
    public static int boxHeightRightBlue = 40;

    public static int topLeftXLeftBlue = 135;
    public static int topLeftYLeftBlue = 25;
    public static int boxWidthLeftBlue = 20;
    public static int boxHeightLeftBlue = 40;

    public int contourTarget = 0;
    public boolean isNormalizing = false;
    private boolean normlizationBroke = false;


    private Rect MIDDLE;

    // Define mats
    Mat ycrcb = new Mat();
    Mat temp = new Mat();


    // Define telemetry variable
    Telemetry telemetry;
    MecDrive drive;

    // Define lists
    private ArrayList<Integer> xList, yList, contourLengths;

    // Define ints
    int cX, cY;
    int maxLength = 0;
    int maxLengthIndex = 0;
    int longestContourX = 0;
    int longestContourY = 0;


    // Don't really know what this thing is, but we're defining it
    Moments M;

    // Enums
    public enum ParkPos {
        LEFT,
        RIGHT,
        CENTER
    }

    public enum AutoSide {
        RED_RIGHT,
        RED_LEFT,
        BLUE_RIGHT,
        BLUE_LEFT
    }

    public enum Mode{
        SLEEVE,
        POLE,
        REDCONE,
        BLUECONE
    }

    // The rectangle/submat used to evaluate the signal color
    static final Rect RIGHT_MIDDLE_RED = new Rect(
            new Point(topLeftXRightRed, topLeftYRightRed),
            new Point(topLeftXRightRed + boxWidthRightRed, topLeftYRightRed + boxHeightRightRed)
    );

    static final Rect LEFT_MIDDLE_RED = new Rect(
            new Point(topLeftXLeftRed, topLeftYLeftRed),
            new Point(topLeftXLeftRed + boxWidthLeftRed, topLeftYLeftRed + boxHeightLeftRed)
    );

    static final Rect RIGHT_MIDDLE_BLUE = new Rect(
            new Point(topLeftXRightBlue, topLeftYRightBlue),
            new Point(topLeftXRightBlue + boxWidthRightBlue, topLeftYRightBlue + boxHeightRightBlue)
    );

    static final Rect LEFT_MIDDLE_BLUE = new Rect(
            new Point(topLeftXLeftBlue, topLeftYLeftBlue),
            new Point(topLeftXLeftBlue + boxWidthLeftBlue, topLeftYLeftBlue + boxHeightLeftBlue)
    );

    public static int topLeftXCone = 75;
    public static int topLeftYCone = 25;
    public static int bottomLeftXCone = 265;
    public static int bottomLeftYCone = 100;

    static final Rect CONE_AREA = new Rect(
            new Point(topLeftXCone, topLeftYCone),
            new Point(bottomLeftXCone, bottomLeftYCone)
    );


    // Sets default values for pipelineMode and position
    //PipelineMode pipelineMode = PipelineMode.SIGNAL;
    Mode sleeveSense = Mode.SLEEVE;
    ParkPos position = ParkPos.CENTER;

    public KevinGodPipelineV2(Telemetry telemetry){
        // Set up lists and telemetry
        xList = new ArrayList<>();
        yList = new ArrayList<>();
        contourLengths = new ArrayList<>();
        this.telemetry = telemetry;
    }

    public KevinGodPipelineV2(Telemetry telemetry, MecDrive drive){
        // Set up lists and telemetry
        xList = new ArrayList<>();
        yList = new ArrayList<>();
        contourLengths = new ArrayList<>();
        this.telemetry = telemetry;
        this.drive = drive;
    }


    public KevinGodPipelineV2(Telemetry telemetry, MecDrive drive, AutoSide autoSide){
        // Set up lists and telemetry
        xList = new ArrayList<>();
        yList = new ArrayList<>();
        contourLengths = new ArrayList<>();
        this.telemetry = telemetry;
        this.drive = drive;
        if(autoSide == AutoSide.RED_LEFT) {
            MIDDLE = LEFT_MIDDLE_RED;
        } else if(autoSide == AutoSide.RED_RIGHT) {
            MIDDLE = RIGHT_MIDDLE_RED;
        } else if(autoSide == AutoSide.BLUE_LEFT) {
            MIDDLE = LEFT_MIDDLE_BLUE;
        } else {
            MIDDLE = RIGHT_MIDDLE_BLUE;
        }
    }

    @Override
    public Mat processFrame(Mat input) {
        // Check pipelineMode and run corresponding image processing
        if(sleeveSense == Mode.SLEEVE) {

            // Convert image to YCrCb color space and extract the Y channel
            Imgproc.cvtColor(input, ycrcb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(ycrcb, temp, 0);
            telemetry.addData("Orig Y", Core.mean(temp.submat(MIDDLE)).val[0]);

            // Make a binary image of values within the desired range and calculate avg color
            Core.inRange(temp, new Scalar(YLower), new Scalar(YUpper), temp);
            double countY = Core.mean(temp.submat(MIDDLE)).val[0];
            int newY = Core.countNonZero(temp);

            // Extract Cr channel
            Core.extractChannel(ycrcb, temp, 1);
            telemetry.addData("Orig Cr", Core.mean(temp.submat(MIDDLE)).val[0]);

            // Make binary image and calculate avg color
            Core.inRange(temp, new Scalar(CrLower), new Scalar(CrUpper), temp);
            double countCr = Core.mean(temp.submat(MIDDLE)).val[0];
            int newCr = Core.countNonZero(temp);




            // Extract Cb channel
            Core.extractChannel(ycrcb, temp, 2);
            telemetry.addData("Orig Cb", Core.mean(temp.submat(MIDDLE)).val[0]);

            // Make binary image and calculate avg color
            Core.inRange(temp, new Scalar(CbLower), new Scalar(CbUpper), temp);
            double countCb = Core.mean(temp.submat(MIDDLE)).val[0];
            int newCb = Core.countNonZero(temp);


            // Telemetry
            telemetry.addData("countY", countY);
            telemetry.addData("countCr", countCr);
            telemetry.addData("countCb", countCb);
            telemetry.addData("newY", newY);
            telemetry.addData("newCr", newCr);
            telemetry.addData("newCb", newCb);

            // Check if certain channels are within certain ranges to determine color
            if(countY > 100 && countCb < 90) {
                telemetry.addData("Color", "Yellow - Left");
                position = ParkPos.LEFT;
            } else if(countCr > 200 /*&& countCb > 200*/) {
                telemetry.addData("Color", "Magenta - Right");
                position = ParkPos.RIGHT;
            } else {
                telemetry.addData("Color", "Cyan - Center ");
                position = ParkPos.CENTER;
            }


            telemetry.update();

            // Display rectangle in detected color
            if(position == ParkPos.LEFT){
                Imgproc.rectangle(
                        input,
                        MIDDLE,
                        new Scalar(255, 255, 0),
                        3

                );

            }else if(position == ParkPos.RIGHT){
                Imgproc.rectangle(
                        input,
                        MIDDLE,
                        new Scalar(255, 0, 255),
                        3

                );

            }else{
                Imgproc.rectangle(
                        input,
                        MIDDLE,
                        new Scalar(0, 155, 255),
                        3

                );
            }

        } else{

            // Convert to HSV color space
            Imgproc.cvtColor(input, temp, Imgproc.COLOR_RGB2HSV);

            // Make binary image of yellow pixels

            if(sleeveSense == Mode.POLE) {
                Core.inRange(temp, new Scalar(H1, S1, V1), new Scalar(H2, S2, V2), temp);
            }else if(sleeveSense == Mode.BLUECONE){
                Core.inRange(temp, new Scalar(H3, S3, V3), new Scalar(H4, S4, V4), temp);
            }else if(sleeveSense == Mode.REDCONE){
                Core.inRange(temp, new Scalar(H5, S5, V5), new Scalar(H6, S6, V6), temp);
            }

            // Blur image to reduce noise
            Imgproc.GaussianBlur(temp, temp, new Size(5, 5), 0);

            // Find all contours in binary image
            List<MatOfPoint> contours = new ArrayList<>();

            if(sleeveSense != Mode.POLE) {
                Imgproc.findContours(temp.submat(CONE_AREA), contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
                Imgproc.rectangle(input, CONE_AREA, new Scalar(255, 92, 90), 2);
            } else {
                Imgproc.findContours(temp, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            }

            for(int i = 0; i < contours.size(); i++){
                // Filter out small, irrelevant contours
                if(contours.get(i).toArray().length > 20) {

                    // Draw all contours to the screen
                    if(sleeveSense != Mode.POLE) {
                        Imgproc.drawContours(input.submat(CONE_AREA), contours, i, new Scalar(230, 191, 254));
                    } else {
                        Imgproc.drawContours(input, contours, i, new Scalar(230, 191, 254));
                    }

                    // Find center of contour and add a point on the screen
                    M = Imgproc.moments(contours.get(i));
                    cX = (int)(M.m10 / M.m00);
                    cY = (int)(M.m01 / M.m00);

                    if(sleeveSense != Mode.POLE) {
                        cX += topLeftXCone;
                        cY += topLeftYCone;
                    }

                    Imgproc.circle(input, new Point(cX, cY), 3, new Scalar(0, 0, 255));

                    // Save the contour's center in a list
                    xList.add(cX);
                    yList.add(cY);

                    // Calculate the length of the contour and add it to a list
                    contourLengths.add(contours.get(i).toArray().length);

                }
            }

            // Reset maxLength so the program doesn't crash - Krish is a genious
            maxLength = 0;

            // Find largest contour
            for(int i = 0; i < xList.size() && i < contourLengths.size() && i < yList.size(); i++) {
                if(contourLengths.get(i) > maxLength) {
                    maxLength = contourLengths.get(i);
                    maxLengthIndex = i;
                }
            }

            // Make sure the program doesn't crash if no contours are found
            if(contourLengths.size() > 0) {
                // Find x coordinate of largest contour and display it on the screen
                longestContourX = xList.get(maxLengthIndex);
                longestContourY = yList.get(maxLengthIndex);
                Imgproc.circle(input, new Point(xList.get(maxLengthIndex), yList.get(maxLengthIndex)), 3, new Scalar(0, 255, 0));
            }

            // Telemetry stuff
            /*telemetry.addData("Contour X Pos", longestContourX);
            telemetry.update();
*/
            // Clear lists
            contourLengths.clear();
            xList.clear();
            yList.clear();

            if(isNormalizing) {
                Imgproc.drawMarker(input, new Point(contourTarget, 120), new Scalar(255, 92, 90));
            } else {
                Imgproc.drawMarker(input, new Point(contourTarget, 120), new Scalar(100, 200, 200));
            }


        }
        return input;
    }

    // Get x coordinate of center of largest contour (pole)
    public int getXContour() {
        return longestContourX;
    }

    public int getPoleYPos(){
        return longestContourY;
    }

    // Get parking position determined by signal mode
    public ParkPos getPosition() {
        return position;
    }

    public int normalize(double power, int target, int tolerance) {
        ElapsedTime time = new ElapsedTime();
        double startTime = time.seconds();
        contourTarget = target;
        isNormalizing = true;
        boolean wrongWay = false;
        int xMax = target + tolerance;
        int xMin = target - tolerance;
        double startPos = drive.avgPos();
        int startPolePosition = getXContour();



        while((getXContour() > xMax || getXContour() < xMin)) {
            if(getXContour() > xMax) {
                drive.setPowerAuto(power, MecDrive.MovementType.ROTATE);
            } else {
                drive.setPowerAuto(-power, MecDrive.MovementType.ROTATE);
            }

//            drive.setPowerAuto(power, MecDrive.MovementType.ROTATE);
            if(time.seconds() - startTime > 2){
                //normlizationBroke = true;
                wrongWay = true;
                break;
            }
        }
        drive.simpleBrake();

        /*
        if(wrongWay) {
            while ((getXContour() > xMax || getXContour() < xMin)) {
            if(getPolePosition() > xMax) {
                drive.setPowerAuto(power, MecDrive.MovementType.ROTATE);
            } else {
                drive.setPowerAuto(-power, MecDrive.MovementType.ROTATE);
            }

                drive.setPowerAuto(-power, MecDrive.MovementType.ROTATE);


            }

            drive.simpleBrake();
        }
        */

        isNormalizing = false;



        if(getXContour() < startPolePosition){
            return -(int)(startPos - drive.avgPos());

        }

        return (int)(startPos - drive.avgPos());



    }

    public int normalizeSpecial(double power, int target, int tolerance) {
        ElapsedTime time = new ElapsedTime();
        double startTime = time.seconds();
        contourTarget = target;
        isNormalizing = true;
        boolean wrongWay = false;
        int xMax = target + tolerance;
        int xMin = target - tolerance;
        double startPos = drive.avgPos();
        int startPolePosition = getXContour();

        if(startPolePosition < xMax){
            power *= -1;
        }

        while((getXContour() > xMax || getXContour() < xMin)) {
            /*if(getPolePosition() > xMax) {
                drive.setPowerAuto(power, MecDrive.MovementType.ROTATE);
            } else {
                drive.setPowerAuto(-power, MecDrive.MovementType.ROTATE);
            }*/

            drive.setPower(power, 0, power, 0);

            if(time.seconds() - startTime > 1.5){
                //normlizationBroke = true;
                wrongWay = true;
                break;
            }
        }
        drive.simpleBrake();

        if(wrongWay) {
            while ((getXContour() > xMax || getXContour() < xMin)) {
            /*if(getPolePosition() > xMax) {
                drive.setPowerAuto(power, MecDrive.MovementType.ROTATE);
            } else {
                drive.setPowerAuto(-power, MecDrive.MovementType.ROTATE);
            }*/

                drive.setPower(-power, 0, -power, 0);


            }

            drive.simpleBrake();
        }

        isNormalizing = false;



        if(getXContour() < startPolePosition){
            return -(int)(startPos - drive.avgPos());

        }

        return (int)(startPos - drive.avgPos());



    }


    public double p = 0.008;
    public int normalizeStrafe(double power, int target, int tolerance) {
        ElapsedTime time = new ElapsedTime();
        double startTime = time.seconds();
        contourTarget = target;
        isNormalizing = true;
        int xMax = target + tolerance;
        int xMin = target - tolerance;
        double startPos = drive.avgPos();
        int startPolePosition = getXContour();
        int error =  target - startPolePosition;
        boolean wrongWay = false;

        /*if(startPolePosition < xMax){
            power *= -1;
        }*/

        while((getXContour() > xMax || getXContour() < xMin)) {
            /*if(getPolePosition() > xMax) {
                drive.setPowerAuto(power, MecDrive.MovementType.ROTATE);
            } else {
                drive.setPowerAuto(-power, MecDrive.MovementType.ROTATE);
            }*/

            power = 0.2;

            drive.setPowerAuto(power, MecDrive.MovementType.STRAFE);

            if(time.seconds() - startTime > 1.5){
                //normlizationBroke = true;
                wrongWay = true;
                break;
            }
        }
        drive.simpleBrake();

        if(wrongWay) {
            while ((getXContour() > xMax || getXContour() < xMin)) {
            /*if(getPolePosition() > xMax) {
                drive.setPowerAuto(power, MecDrive.MovementType.ROTATE);
            } else {
                drive.setPowerAuto(-power, MecDrive.MovementType.ROTATE);
            }*/

                drive.setPowerAuto(-power, MecDrive.MovementType.STRAFE);


            }
            drive.simpleBrake();

        }

        isNormalizing = false;



        if(getXContour() < startPolePosition){
            return -(int)(startPos - drive.avgPos());

        }

        return (int)(startPos - drive.avgPos());



    }
    public boolean getNormalizationBroke(){
        return normlizationBroke;
    }



    public void changeMode(Mode mode){
        sleeveSense = mode;
    }

    public void Ynormalize(double power, int target, int tolerance){
        int yMax = target + tolerance;
        int yMin = target - tolerance;
        while(getPoleYPos() > yMax || getPoleYPos() < yMin) {
            if(getPoleYPos() > yMax) {
                drive.setPowerAuto(power, MecDrive.MovementType.STRAIGHT);
            } else {
                drive.setPowerAuto(-power, MecDrive.MovementType.ROTATE);
            }
        }
    }



}
