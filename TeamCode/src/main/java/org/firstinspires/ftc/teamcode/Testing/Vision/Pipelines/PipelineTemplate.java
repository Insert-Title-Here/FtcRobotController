package org.firstinspires.ftc.teamcode.Testing.Vision.Pipelines;

import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvPipeline;

public class PipelineTemplate extends OpenCvPipeline {

    @Override
    public Mat processFrame(Mat input) {
        return input;
    }
}
