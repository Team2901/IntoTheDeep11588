package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.TouchSensor;

public class MockTouchSensor implements TouchSensor {
    @Override
    public double getValue() {
        return 0;
    }

    @Override
    public boolean isPressed() {
        return false;
    }

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }

    @Override
    public String getDeviceName() {
        return null;
    }

    @Override
    public String getConnectionInfo() {
        return null;
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }
}
