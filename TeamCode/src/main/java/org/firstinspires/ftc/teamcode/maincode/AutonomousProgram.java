package org.firstinspires.ftc.teamcode.maincode;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.BaseAutonomous;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.util.annotations.LinkTo;
import com.SCHSRobotics.HAL9001.util.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous", group = "competition")
@LinkTo(destination = "Teleop")
public class AutonomousProgram extends BaseAutonomous {
    private Cygnus robot;
    @Override
    public Robot buildRobot() {
        robot = new Cygnus(this);
        return robot;
    }

    @Override
    public void main() {
        robot.drive.drive(new Vector(0,0.2));
        waitUntil(() -> robot.lineDetector.isRedDetected() || robot.lineDetector.isBlueDetected());
        robot.drive.stopAllMotors();
    }
}
