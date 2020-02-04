package org.firstinspires.ftc.teamcode.maincode;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.BaseAutonomous;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.util.math.Vector;

public class AutonomousProgramBlue extends BaseAutonomous {
    private Cygnus robot;
    @Override
    public Robot buildRobot() {
        robot = new Cygnus(this);
        return robot;
    }

    @Override
    public void main() {
        robot.drive.drive(new Vector(0,0.2));
        while(opModeIsActive() && (!robot.lineDetector.isBlueDetected())) {
            waitFor(1);
        }
        robot.drive.stopAllMotors();
    }
}
