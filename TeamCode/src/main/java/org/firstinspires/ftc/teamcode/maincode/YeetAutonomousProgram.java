package org.firstinspires.ftc.teamcode.maincode;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.BaseAutonomous;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.util.annotations.StandAlone;
import com.SCHSRobotics.HAL9001.util.math.Units;
import com.SCHSRobotics.HAL9001.util.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@StandAlone
@Autonomous(name = "YeetAutonomous", group = "competition")
public class YeetAutonomousProgram extends BaseAutonomous {
    private Cygnus robot;
    @Override
    public Robot buildRobot() {
        robot = new Cygnus(this);
        return robot;
    }

    @Override
    public void main() {
        robot.drive.driveDistance(new Vector(0.5,0),2, Units.TILES);
        robot.intake.intake(1);
        robot.drive.driveDistance(new Vector(0,0.3),0.5, Units.TILES);
        robot.intake.stopIntake();
        robot.drive.driveDistance(new Vector(-0.5,0),2,Units.TILES);
        robot.drive.drive(new Vector(0,0.2));
        while (!robot.lineDetector.isRedDetected()) {

        }
        robot.drive.stopAllMotors();
    }
}
