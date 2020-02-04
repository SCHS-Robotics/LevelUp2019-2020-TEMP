package org.firstinspires.ftc.teamcode.maincode;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class TestingRobot extends Robot {
    public Intake intake;
    public LineDetector lineDetector;
    public TestingRobot(OpMode opmode) {
        super(opmode);
        //intake = new Intake(this, "intakeLeft", "intakeRight", "stackIntakeLeft", new Button(1, Button.BooleanInputs.bool_left_trigger), new Button(1, Button.BooleanInputs.bool_right_trigger), 1);
        lineDetector = new LineDetector(this, "colorSensor");
        putSubSystem("Intake", intake);
        putSubSystem("lineSensor", lineDetector);
    }
}
