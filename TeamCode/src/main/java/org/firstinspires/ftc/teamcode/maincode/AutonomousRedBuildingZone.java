package org.firstinspires.ftc.teamcode.maincode;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.BaseAutonomous;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.util.math.Vector;

public class AutonomousRedBuildingZone extends BaseAutonomous {
    private Cygnus robot;
    @Override
    public Robot buildRobot() {
        robot = new Cygnus(this);
        return robot;
    }

    @Override
    public void main() {


        robot.drive.driveEncoders(new Vector(0.1, -0.25), 1500);
        sleep(1000);
        robot.mover.latch();
        sleep(1000);

        robot.drive.turnEncoders(1, 900);
        sleep(1000);
        robot.drive.driveEncoders(new Vector(-0.1, 0), 300);
        sleep(1000);
        robot.mover.resetLatch();
        sleep(1000);
        robot.drive.driveEncoders(new Vector(0.05, -0.25), 2300);
        //Move into subcase 1 or 2.

    }
}
