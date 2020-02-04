package org.firstinspires.ftc.teamcode.maincode;

import android.graphics.Color;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.SubSystem;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class colorSensor extends SubSystem {
    public ColorSensor color_sensor = robot.hardwareMap.get(ColorSensor.class, "colorSensor");
    public DistanceSensor distance_sensor = robot.hardwareMap.get(DistanceSensor.class, "colorSensor");
    public colorSensor(Robot robot) {
        super(robot);

    }
    /*YEETben had a car that had a cat that ate marshmellows but they were really deadly so ben cried YEEET,*/
    @Override
    public void init() {

    }

    @Override
    public void handle() {
        double dist = distance_sensor.getDistance(DistanceUnit.CM);
        double r = color_sensor.red();
        double g = color_sensor.green();
        double b = color_sensor.blue();
        double a = color_sensor.alpha();

        float hsvValues[] = new float[3];

        Color.RGBToHSV(255 * color_sensor.red(), 255 * color_sensor.green(), 255 * color_sensor.blue(), hsvValues);

        robot.telemetry.clearAll();
        robot.telemetry.addData("h", hsvValues[0]);
        robot.telemetry.addData("s", hsvValues[1]);
        robot.telemetry.addData("v", hsvValues[2]);
        robot.telemetry.update();
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {

    }

    public float[] getHSV() {

        float hsvValues[] = new float[3];

        Color.RGBToHSV(255 * color_sensor.red(), 255 * color_sensor.green(), 255 * color_sensor.blue(), hsvValues);
        return hsvValues;
    }
    public double getDist() {
        return distance_sensor.getDistance(DistanceUnit.CM);
    }
}
