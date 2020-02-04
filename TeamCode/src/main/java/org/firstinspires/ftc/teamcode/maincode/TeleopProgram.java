package org.firstinspires.ftc.teamcode.maincode;

import android.media.MediaPlayer;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.BaseTeleop;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.util.misc.Button;
import com.SCHSRobotics.HAL9001.util.misc.CustomizableGamepad;
import com.SCHSRobotics.HAL9001.util.misc.Toggle;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.R;

@TeleOp(name = "Teleop", group = "competition")
public class TeleopProgram extends BaseTeleop {

    private static final String CCWBUTTON = "ccwButton", CWBUTTON = "cwButton", TURNAROUNDBUTTON = "turnAroundButton", XBUTTON = "xbutton";
    private Cygnus robot;

    private MediaPlayer beatBox;

    private Toggle turnCCWToggle = new Toggle(Toggle.ToggleTypes.trueOnceToggle, false);
    private Toggle turnCWToggle = new Toggle(Toggle.ToggleTypes.flipToggle, false);
    private Toggle turnAroundToggle = new Toggle(Toggle.ToggleTypes.flipToggle, false);

    private CustomizableGamepad gamepad;

    private final Button turnCCWShortcut = new Button(1, Button.BooleanInputs.x);
    private final Button turnCWShortcut = new Button(1, Button.BooleanInputs.a);
    private final Button turnAroundShortcut = new Button(1, Button.BooleanInputs.b);
    private final Button turnxButton = new Button(1, Button.BooleanInputs.x);

    @Override
    public Robot buildRobot() {
        robot = new Cygnus(this);

        gamepad = new CustomizableGamepad(robot);
        gamepad.addButton(CCWBUTTON, turnCCWShortcut);
        gamepad.addButton(CWBUTTON, turnCWShortcut);
        gamepad.addButton(TURNAROUNDBUTTON, turnAroundShortcut);
        gamepad.addButton(XBUTTON, turnxButton);

        return robot;
    }

    @Override
    public void onUpdate() {
        turnCCWToggle.updateToggle(gamepad.getBooleanInput(CCWBUTTON));
        turnCWToggle.updateToggle(gamepad.getBooleanInput(CWBUTTON));
        turnAroundToggle.updateToggle(gamepad.getBooleanInput(TURNAROUNDBUTTON));


    }
}
