package org.firstinspires.ftc.teamcode.maincode;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.SubSystem;
import com.SCHSRobotics.HAL9001.util.annotations.TeleopConfig;
import com.SCHSRobotics.HAL9001.util.misc.Button;
import com.SCHSRobotics.HAL9001.util.misc.ConfigParam;
import com.SCHSRobotics.HAL9001.util.misc.CustomizableGamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class FoundationMover extends SubSystem {

    private static final String LATCH = "latch";
    private Servo latch;
    private Servo latch2;
    private CustomizableGamepad gamepad;
    public FoundationMover(Robot robot, String latchConfigLeft, String latchConfigRight, Button plopButton) {
        super(robot);
        latch = robot.hardwareMap.servo.get(latchConfigLeft);
        latch2 = robot.hardwareMap.servo.get(latchConfigRight);

        latch.setDirection(Servo.Direction.REVERSE);
        latch2.setDirection(Servo.Direction.FORWARD);

        gamepad = new CustomizableGamepad(robot);
        gamepad.addButton(LATCH, plopButton);
    }

    public FoundationMover(Robot robot, String latchConfigLeft, String latchConfigRight) {
        super(robot);
        latch = robot.hardwareMap.servo.get(latchConfigLeft);
        latch2 = robot.hardwareMap.servo.get(latchConfigRight);

        latch.setDirection(Servo.Direction.REVERSE);
        latch2.setDirection(Servo.Direction.FORWARD);

        gamepad = new CustomizableGamepad(robot);

        usesConfig = true;
    }

    @Override
    public void init() {
        resetLatch();
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        if(usesConfig) {
            gamepad = robot.pullControls(this);
            //Log.wtf("test0",Robot.teleopConfig.get(this).toString());
        }
    }

    @Override
    public void handle() {
        if(gamepad.getBooleanInput(LATCH)) {
            latch();
        }
        else {
            resetLatch();
        }
    }

    @Override
    public void stop() {

    }

    public void setLatch(double position) {
        latch.setPosition(position);
        latch2.setPosition(position);
    }

    public void latch() {
        setLatch(0);
    }

    public void resetLatch() {
        setLatch(0.5);
    }

    public void latchTime(long time) {
        latch();
        waitTime(time);
        resetLatch();
    }

    @TeleopConfig
    public static ConfigParam[] teleopConfig() {
        return new ConfigParam[] {
                new ConfigParam(LATCH, Button.BooleanInputs.b)
        };
    }
}
