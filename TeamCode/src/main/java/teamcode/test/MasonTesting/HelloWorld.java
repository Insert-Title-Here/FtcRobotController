package teamcode.test.MasonTesting;

import teamcode.common.AbstractOpMode;

public class HelloWorld extends AbstractOpMode {

    @Override
    protected void onInitialize() {

    }

    @Override
    protected void onStart() {
        telemetry.addData("Hello world", "");
        telemetry.update();
    }

    @Override
    protected void onStop() {

    }
}
