package org.firstinspires.ftc.teamcode.maincode;

import android.media.MediaPlayer;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.BaseAutonomous;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.util.annotations.MainRobot;
import com.SCHSRobotics.HAL9001.util.math.Vector;
import com.SCHSRobotics.HAL9001.util.misc.BeatBox;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;


/**
 * Place the bot with the intake touching the wall and the left wheels on the crease
 */
@Autonomous(name = "AutonomousRedBuildingWallNeutral", group = "competition")
public class AutonomousRedBuildingZone extends BaseAutonomous {
    public @MainRobot Cygnus robot;
    public SampleMecanumDriveREVOptimized drive;
    private BeatBox beatBox;

    @Override
    public void main() {

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .back(5)
                        .build()
        );
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeRight(12)
                        .build()
        );
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .back(27)
                        .build()
        );
        waitTime(200);
        robot.mover.latch();
        waitTime(300);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .forward(36)
                        .build()
        );
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .back(2.5)
                        .build()
        );
        robot.mover.resetLatch();
        waitTime(500);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeLeft(36)
                        .build()
        );
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .back(18)
                        .build()
        );
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeRight(15)
                        .build()
        );
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .back(10.5)
                        .build()
        );
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeLeft(28)
                        .build()
        );



    }
    @Override
    public void onInit() {
        drive = new SampleMecanumDriveREVOptimized(robot.hardwareMap);
        robot.mover.resetLatch();
        robot.hugger.hugLeft();
        beatBox = new BeatBox();
        beatBox.addSong("Spooky", MediaPlayer.create(robot.hardwareMap.appContext, R.raw.spookyskeleboys));
        beatBox.baseBoost("Spooky",100);

        beatBox.playSong("Spooky");

        //robot.skystoneDetector.startVision();
    }

    @Override
    public void onStop() {
        beatBox.stopSong("Spooky");
    }
}
