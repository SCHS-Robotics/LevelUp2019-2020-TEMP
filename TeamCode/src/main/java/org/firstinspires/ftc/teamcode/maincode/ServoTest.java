package org.firstinspires.ftc.teamcode.maincode;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.SubSystem;
import com.SCHSRobotics.HAL9001.util.misc.Button;
import com.SCHSRobotics.HAL9001.util.misc.CustomizableGamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoTest extends SubSystem {

    private static final String SERVO_ZERO = "zero", SERVO_ONE = "one", SERVO_MID = "mid";

    private Servo plop;

    private CustomizableGamepad gamepad;
    public ServoTest(Robot robot, String servoConfig) {
        super(robot);
        plop = robot.hardwareMap.servo.get(servoConfig);
        gamepad = new CustomizableGamepad(robot);
        gamepad.addButton(SERVO_ZERO, new Button(1,Button.BooleanInputs.x));
        gamepad.addButton(SERVO_MID, new Button(1,Button.BooleanInputs.y));
        gamepad.addButton(SERVO_ONE, new Button(1,Button.BooleanInputs.b));
    }

    @Override
    public void init() {

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void handle() {
        if(gamepad.getBooleanInput(SERVO_ZERO)) {
            plop.setPosition(0);
        }
        else if(gamepad.getBooleanInput(SERVO_MID)) {
            plop.setPosition(0.5);
        }
        else if(gamepad.getBooleanInput(SERVO_ONE)) {
            plop.setPosition(1);
        }
        robot.telemetry.addLine(Double.toString(plop.getPosition()));
        robot.telemetry.update();
    }

    @Override
    public void stop() {
        plop.setPosition(0);
    }
}
