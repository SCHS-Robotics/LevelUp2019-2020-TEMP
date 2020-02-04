package org.firstinspires.ftc.teamcode.maincode;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.BaseAutonomous;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.util.math.Units;
import com.SCHSRobotics.HAL9001.util.math.Vector;

public class AutonomousRedBuildingZoneWall extends BaseAutonomous {
    private Cygnus robot;
    @Override
    public Robot buildRobot() {
        robot = new Cygnus(this);
        return robot;
    }

    @Override
    public void main() {
        /*rotates 90 degrees clockwise

         * moves towards foundation
         * grabs foundation
         * move back to start point
         * rotates 90 degrees clockwise putting foundation in site
         * moves back into parking case 1 or two*/
        robot.drive.turnEncoders(0.5, Math.PI/2);
        robot.drive.driveDistance(new Vector(-0.5, 0), 100, Units.CENTIMETERS);
        robot.mover.latch();
        robot.drive.driveDistance(new Vector(0.5, 0), 100, Units.CENTIMETERS);
        robot.drive.turnEncoders(0.5, Math.PI/2);
        robot.mover.resetLatch();
        //Move into subcase 1 or 2.

    }
}
