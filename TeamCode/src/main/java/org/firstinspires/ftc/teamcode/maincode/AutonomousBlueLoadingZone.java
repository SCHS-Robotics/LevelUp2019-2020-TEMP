package org.firstinspires.ftc.teamcode.maincode;


import android.media.MediaPlayer;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.BaseAutonomous;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.util.annotations.MainRobot;
import com.SCHSRobotics.HAL9001.util.control.PIDController;
import com.SCHSRobotics.HAL9001.util.math.Vector;
import com.SCHSRobotics.HAL9001.util.misc.BeatBox;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.path.heading.TangentInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.opencv.core.Point;

import java.util.List;

import kotlin.Unit;

import static java.lang.Math.PI;
import static java.lang.Math.toRadians;

@Autonomous(name = "AutonomousBlueLoading", group = "competition")
public class AutonomousBlueLoadingZone extends BaseAutonomous {
    public @MainRobot Cygnus robot;
    public SampleMecanumDriveREVOptimized drive;

    private BeatBox beatBox;

    private static final double LEFT_DIVIDER = 118, RIGHT_DIVIDER = 182;
    private enum StoneState {
        LEFT(-4), CENTER(4), RIGHT(12);

        public double dX;
        StoneState(double dX) {
            this.dX = dX;
        }
    }
    private StoneState state;

    @Override
    public void main() {
        /*moves down left  below stones
         * moves up and intakes / stores a stone
         * moves back out of stones
         * strafes right then drive into subcase 1 or 2
        robot.drive.driveEncoders(new Vector(0.3,Math.toRadians(0), Vector.CoordinateType.POLAR), 1550);
        waitTime(500);
        robot.hugger.hug();
        waitTime(500);
        robot.drive.driveEncoders(new Vector(-0.5,Math.toRadians(-10), Vector.CoordinateType.POLAR), 2200);
        waitTime(500);
        robot.drive.driveEncoders(new Vector(0.3,Math.toRadia+ns(0), Vector.CoordinateType.POLAR), 100);
        waitTime(500);
        robot.drive.driveEncoders(new Vector(-0.3,Math.toRadians(-90), Vector.CoordinateType.POLAR), 2600);
        waitTime(500);
        robot.hugger.reset();
        waitTime(500);
        robot.drive.drive(new Vector(-0.1,Math.toRadians(90), Vector.CoordinateType.POLAR));
        waitWhile(() -> !robot.lineDetector.isBlueDetected());
        robot.drive.stopAllMotors();*/

        robot.telemetry.setAutoClear(true);

        waitUntil(() -> robot.skystoneDetector.getSkystones().size() > 0);
        robot.skystoneDetector.stopVision();

        robot.telemetry.addLine("Skystone Detected");
        robot.telemetry.update();

        List<Point> skystones = robot.skystoneDetector.getSkystones();
        Point skystoneLoc;
        if(skystones.size() == 1) {
            skystoneLoc = skystones.get(0);
        }
        else {
            skystoneLoc = skystones.get(0).x < skystones.get(1).x ? skystones.get(0) : skystones.get(1);
        }

        if(skystoneLoc.x < LEFT_DIVIDER) {
            state = StoneState.LEFT;
        }
        else if(skystoneLoc.x >= LEFT_DIVIDER && skystoneLoc.x <= RIGHT_DIVIDER) {
            state = StoneState.CENTER;
        }
        else {
            state = StoneState.RIGHT;
        }

        telemetry.addData("state", state.name());
        telemetry.update();
        if (state.dX < 0) {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .forward(-state.dX)//splineTo(new Pose2d(32.25, 24   , toRadians(90))))
                            .build()

            );
        } else {
            drive.followTrajectorySync(
                    drive.trajectoryBuilder()
                            .back(state.dX)//splineTo(new Pose2d(32.25, 24   , toRadians(90))))
                            .build()


            );
        }

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeRight(36.5)//splineTo(new Pose2d(32.25, 24   , toRadians(90))))
                        .build()

        );

        robot.hugger.hug();
        waitTime(200);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeLeft(13)
                        .build()
        );
        waitTime(200);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .setReversed(true)
                        .splineTo(new Pose2d(-50,-24, toRadians(0)))
                        .build()
        );

        robot.hugger.reset();
        waitTime(200);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(23+state.dX,-30, toRadians(0)))
                        .build()

        );

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeRight(3)
                        .build()

        );

        robot.hugger.hug();
        waitTime(500);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeLeft(13.5)
                        .build()
        );

        waitTime(200);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                .setReversed(true)
                .splineTo(new Pose2d(-50,-23.5, toRadians(0)))
                .build()
        );
        robot.hugger.reset();
        waitTime(200);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .splineTo(new Pose2d(10+state.dX,-30, toRadians(0)))
                        .build()

        );
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeRight(3)
                        .build()

        );
        robot.hugger.hug();

        waitTime(200);

        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .strafeLeft(13.5)
                        .build()
        );

        waitTime(200);
        drive.followTrajectorySync(
                drive.trajectoryBuilder()
                        .setReversed(true)
                        .splineTo(new Pose2d(-42,-20, toRadians(0)))
                        .build()
        );

        robot.hugger.reset();

        //robot.intake.intake(1);
        //robot.drive.driveDistance(new Vector(0,0.5), 25, Units.CENTIMETERS);
        //robot.intake.intake(0);
        //robot.drive.driveDistance(new Vector(-0.5,0), 25, Units.CENTIMETERS);
        //moves into subcase 1 or 2
    }

    @Override
    protected void onInit() {
        drive = new SampleMecanumDriveREVOptimized(robot.hardwareMap);

        beatBox = new BeatBox();
        beatBox.addSong("Spooky", MediaPlayer.create(robot.hardwareMap.appContext,R.raw.wenumberone));
        beatBox.baseBoost("Spooky",100);

        //beatBox.playSong("Spooky");

        robot.skystoneDetector.startVision();
    }

    @Override
    protected void onStop() {
        beatBox.stopSong("Spooky");
    }
}
