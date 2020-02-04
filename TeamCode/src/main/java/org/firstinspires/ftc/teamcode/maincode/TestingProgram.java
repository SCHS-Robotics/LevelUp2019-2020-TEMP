package org.firstinspires.ftc.teamcode.maincode;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.BaseTeleop;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.util.annotations.StandAlone;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@StandAlone
@TeleOp(name = "ServoTester", group = "Testing")
public class TestingProgram extends BaseTeleop {
    private TestingBot robot;
    @Override
    protected Robot buildRobot() {
        robot = new TestingBot(this);
        return robot;
    }
}
