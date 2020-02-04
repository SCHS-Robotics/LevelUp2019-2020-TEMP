package org.firstinspires.ftc.teamcode.maincode;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.SubSystem;
import com.SCHSRobotics.HAL9001.util.annotations.TeleopConfig;
import com.SCHSRobotics.HAL9001.util.misc.Button;
import com.SCHSRobotics.HAL9001.util.misc.ConfigParam;
import com.SCHSRobotics.HAL9001.util.misc.CustomizableGamepad;
import com.qualcomm.robotcore.hardware.Servo;

public class CapstonePlopper extends SubSystem {

    private static final String PLOP = "plop";
    private Servo plop;
    private CustomizableGamepad gamepad;
    public CapstonePlopper(Robot robot, String plopperConfig, Button plopButton) {
        super(robot);
        plop = robot.hardwareMap.servo.get(plopperConfig);
        gamepad = new CustomizableGamepad(robot);

        gamepad.addButton(PLOP, plopButton);
    }

    public CapstonePlopper(Robot robot, String plopperConfig) {
        super(robot);
        plop = robot.hardwareMap.servo.get(plopperConfig);
        gamepad = new CustomizableGamepad(robot);

        usesConfig = true;
    }

    @Override
    public void init() {
        resetPlop();
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        if(usesConfig) {
            gamepad = robot.pullControls(this);
        }
    }

    @Override
    public void handle() {
        if(gamepad.getBooleanInput(PLOP)) {
            plop();
        }
        else {
            resetPlop();
        }
    }

    @Override
    public void stop() {

    }

    public void setPlop(double position) {
        plop.setPosition(position);
    }

    public void plop() {
        setPlop(0.7);
    }

    public void resetPlop() {
        setPlop(0);
    }

    public void plopTime(long time) {
        plop();
        waitTime(time);
        resetPlop();
    }

    @TeleopConfig
    public static ConfigParam[] teleopConfig() {
        return new ConfigParam[] {
                new ConfigParam(PLOP, Button.BooleanInputs.a)
        };
    }
}
