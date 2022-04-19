package teamcode.test.Miscellanious;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cConfigureChannelCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.configuration.annotations.DigitalIoDeviceType;

import org.openftc.revextensions2.ExpansionHubEx;

import teamcode.common.AbstractOpMode;
import teamcode.common.Logger;
import teamcode.common.Utils;

@Disabled
@Autonomous(name="Bus Test")
public class BusSpeedTest extends AbstractOpMode {
    ExpansionHubEx hub;
    Logger logger;
    ColorRangeSensor sensor;
    NormalizedRGBA rgba;
    NormalizedColorSensor color;
    LynxModule eh;
    private long runInterval = (long)Math.round(1.0/250.0 * 1000.0);


    @Override
    protected void onInitialize() {
        eh = hardwareMap.get(LynxModule.class, "Control Hub");
        hub = hardwareMap.get(ExpansionHubEx.class, "Control Hub");
        //sensor = hardwareMap.get(ColorRangeSensor.class, "color");
        color = hardwareMap.get(NormalizedColorSensor.class, "color");
        logger = new Logger(new String[]{"STANDARD", "FAST", "FAST_PLUS", "LUDICROUS_SPEED"});
    }

    @Override
    protected void onStart() {

        double start = time;
        double delta = time - start;
        double iterator = 0;
        double deltaReadSum = 0;
        while(delta < 1.0){
            double startTime = time;
            rgba = color.getNormalizedColors();
            double deltaRead = time - startTime;
            deltaReadSum += deltaRead;
            //logger.writeToLogString(0, iterator +", " + rgba.red  + "\n");
            delta = time - start;
            iterator++;
        }
        telemetry.addData("standard", deltaReadSum / iterator);
        telemetry.addData("iterator", iterator);

        hub.setAllI2cBusSpeeds(ExpansionHubEx.I2cBusSpeed.FAST_400K);
        Utils.sleep(5000);
        start = time;
        delta = time - start;
        iterator = 0;
        deltaReadSum = 0;
        while(delta < 1.0){
            double startTime = time;
            rgba = color.getNormalizedColors();
            double deltaRead = time - startTime;
            deltaReadSum += deltaRead;
            //logger.writeToLogString(1, iterator +", " + rgba.red  + "\n");
            delta = time - start;
            iterator++;
        }
        telemetry.addData("fast", deltaReadSum / iterator);
        telemetry.addData("iterator", iterator);


        hub.setAllI2cBusSpeeds(ExpansionHubEx.I2cBusSpeed.FASTPLUS_1M);
        Utils.sleep(5000);
        start = time;
        delta = time - start;
        iterator = 0;
        deltaReadSum = 0;
        while(delta < 1.0){
            double startTime = time;
            rgba = color.getNormalizedColors();
            double deltaRead = time - startTime;
            deltaReadSum += deltaRead;
            //logger.writeToLogString(2, iterator +", " + rgba.red  + "\n");
            delta = time - start;
            iterator++;
        }
        telemetry.addData("Fast Plus", deltaReadSum / iterator);
        telemetry.addData("iterator", iterator);


        hub.setAllI2cBusSpeeds(ExpansionHubEx.I2cBusSpeed.HIGH_3_4M);
        Utils.sleep(5000);
        start = time;
        delta = time - start;
        iterator = 0;
        deltaReadSum = 0;
        while(delta < 1.0){
            double startTime = time;
            rgba = color.getNormalizedColors();
            double deltaRead = time - startTime;
            deltaReadSum += deltaRead;
            //logger.writeToLogString(3, iterator +", " + rgba.red  + "\n");
            delta = time - start;
            iterator++;
        }
        telemetry.addData("High", deltaReadSum / iterator);
        telemetry.addData("iterator", iterator);
        telemetry.update();
        while(opModeIsActive());

    }


    public synchronized void setBusSpeed(LynxI2cConfigureChannelCommand.SpeedCode speed){
        for(int i = 0; i < 4; i++) {
            LynxI2cConfigureChannelCommand command = new LynxI2cConfigureChannelCommand(eh, i, speed);
            try {
                command.send();
            } catch (InterruptedException e) {
                e.printStackTrace();
            } catch (LynxNackException e) {
                e.printStackTrace();
            }
        }
    }

    @Override
    protected void onStop() {
        //logger.writeLoggerToFile();
    }
}
