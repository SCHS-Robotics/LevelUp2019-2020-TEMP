package org.firstinspires.ftc.teamcode.maincode;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.BaseAutonomous;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.util.math.Units;
import com.SCHSRobotics.HAL9001.util.math.Vector;

public class AutonomousRedLoadingZoneWall extends BaseAutonomous {
    private Cygnus robot;
    @Override
    public Robot buildRobot() {
        robot = new Cygnus(this);
        return robot;
    }

    @Override
    public void main() {
        /*moves down left  below stones
         * moves up and intakes / stores a stone
         * moves back out of stones
         * strafes right then drive into subcase 1 or 2 */
        robot.drive.driveDistance(new Vector(-0.5,-0.5), 100, Units.CENTIMETERS);
        robot.intake.intake(1);
        robot.drive.driveDistance(new Vector(0,0.5), 25, Units.CENTIMETERS);
        robot.intake.intake(0);
        robot.drive.driveDistance(new Vector(0.5,0), 25, Units.CENTIMETERS);
        //moves into subcase 1 or 2
    }
}
