package org.firstinspires.ftc.teamcode.maincode;

import android.graphics.Color;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.SubSystem;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class LineDetector extends SubSystem {

    private ColorSensor colorSensor;

    private static final double BLUE_CENTER = 210;

    private static double BLUE_RADIUS = 15.0;
    private static double RED_RADIUS = 15.0;

    public LineDetector(Robot robot, String colorSensorConfig) {
        super(robot);
        colorSensor = robot.hardwareMap.colorSensor.get(colorSensorConfig);
    }

    @Override
    public void init() {
        enableLed();
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void handle() {

    }

    @Override
    public void stop() {
        disableLed();
    }

    public void enableLed() {
        colorSensor.enableLed(true);
    }

    public void disableLed() {
        colorSensor.enableLed(false);
    }

    public double getColor() {
        double[] hsv = getHSV();
        return hsv[0];
    }
    public double[] getHSV() {
        float[] hsvFloat = new float[3];
        double[] hsv = new double[3];
        Color.RGBToHSV((colorSensor.red() * 255), (colorSensor.green() * 255), (colorSensor.blue() * 255), hsvFloat);
        for (int i = 0; i < hsv.length; i++) {
            hsv[i] = (double) hsvFloat[i];
        }
        return hsv;
    }

    public double[] getRGB() {
        return new double[] {colorSensor.red(),colorSensor.green(),colorSensor.blue()};
    }

    public boolean isRedDetected() {
        double color = getColor();
        return color < RED_RADIUS || color > 360 - RED_RADIUS;
    }

    public boolean isBlueDetected() {
        double color = getColor();
        return Math.abs(color - BLUE_CENTER) < BLUE_RADIUS;
    }

    public void setRedRadius(double redRadius) {
        RED_RADIUS = redRadius;
    }

    public void setBlueRadius(double blueRadius) {
        BLUE_RADIUS = blueRadius;
    }
}
