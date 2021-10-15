package teamcode.test.CVNew;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class MineralTrackerPipeline extends OpenCvPipeline {


    @Override
    public Mat processFrame(Mat input) {

        Mat leftHalf = input.submat(0, input.rows() / 2, 0, input.cols() );
        Mat rightHalf = input.submat(input.rows() / 2, input.rows(), 0, input.cols());

        return null;
    }
}
