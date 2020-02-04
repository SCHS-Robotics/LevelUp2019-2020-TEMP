package org.firstinspires.ftc.teamcode.maincode;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.SubSystem;
import com.SCHSRobotics.HAL9001.util.annotations.TeleopConfig;
import com.SCHSRobotics.HAL9001.util.misc.Button;
import com.SCHSRobotics.HAL9001.util.misc.ConfigParam;
import com.SCHSRobotics.HAL9001.util.misc.CustomizableGamepad;
import com.SCHSRobotics.HAL9001.util.misc.Toggle;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.jetbrains.annotations.NotNull;

public class Hugger extends SubSystem {
    private Toggle hugToggle = new Toggle(Toggle.ToggleTypes.flipToggle, false);
    private Servo hugger;
    private CustomizableGamepad gamepad;
    public Hugger(@NotNull Robot robot, String huggerServoConfig) {
        super(robot);
        hugger = robot.hardwareMap.servo.get(huggerServoConfig);
        gamepad = new CustomizableGamepad(robot);
        usesConfig = true;
    }

    @Override
    public void init() {
        reset();
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        if (usesConfig) {
            gamepad = robot.pullControls(this);
        }
    }

    @Override
    public void handle() {
        hugToggle.updateToggle(gamepad.getBooleanInput("Hugger"));
        if (hugToggle.getCurrentState())
            hug();
        else
            reset();
    }

    @Override
    public void stop() {

    }

    public void hug() {
        setHuggerPos(1);
    }

    public void hugTime(long timeMillis) {
        hug();
        waitTime(timeMillis);
        reset();
    }

    public void reset() {
        setHuggerPos(0.5);
    }

    public void setHuggerPos(double pos) {
        hugger.setPosition(Range.clip(pos,0,1));
    }

    @TeleopConfig
    public static ConfigParam[] teleopConfig() {
        return new ConfigParam[] {
                new ConfigParam("Hugger", Button.BooleanInputs.dpad_down),
        };
    }
}
