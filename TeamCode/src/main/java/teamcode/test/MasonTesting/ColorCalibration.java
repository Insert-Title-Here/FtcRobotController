package teamcode.test.MasonTesting;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ColorCalibration extends OpenCvPipeline {

    Mat HSV = new Mat();
    Mat c1 = new Mat();
    Mat c2 = new Mat();
    Mat c3 = new Mat();
    Mat hsvRegion;
    int avgH; int avgS; int avgV;
    final int font = Imgproc.FONT_HERSHEY_COMPLEX;

    void inputToHSV(Mat input, int column, Mat dst) {
        Core.extractChannel(input, dst, column);
    }

    @Override
    public void init(Mat input) {
        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);

        inputToHSV(HSV, 0, c1);
        inputToHSV(HSV, 1, c2);
        inputToHSV(HSV, 2, c3);

        hsvRegion = HSV.submat(new Rect(new Point(50, 50), new Point(55, 55)));
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, HSV, Imgproc.COLOR_RGB2HSV);

        inputToHSV(HSV, 0, c1);
        inputToHSV(HSV, 1, c2);
        inputToHSV(HSV, 2, c3);

        hsvRegion = HSV.submat(new Rect(new Point(50, 50), new Point(55, 55)));

        avgH = (int) Core.mean(hsvRegion).val[0];
        avgS = (int) Core.mean(hsvRegion).val[1];
        avgV = (int) Core.mean(hsvRegion).val[2];

        Imgproc.rectangle(input, new Point(50, 50), new Point(55, 55), new Scalar(255, 255, 255));
        String text = "RGB: " + avgH + " " + avgS + " " + avgV;
        // 320, 240
        Imgproc.putText(input, text, new Point(20, 200), font, 1, new Scalar(255, 255, 255), 3);

        return input;
    }

    public int hValue() {
        return avgH;
    }

    public int sValue() {
        return avgS;
    }

    public int vValue() {
        return avgV;
    }

}
