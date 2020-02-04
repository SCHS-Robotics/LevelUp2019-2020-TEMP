package org.firstinspires.ftc.teamcode.maincode;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.system.subsystems.MechanumDrive;
import com.SCHSRobotics.HAL9001.util.calib.AnglePIDTunerSystem;
import com.SCHSRobotics.HAL9001.util.control.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class TestingBot extends Robot {
    public AnglePIDTunerSystem tuner;
    public TestingBot(OpMode opMode) {
        super(opMode);
        tuner = new AnglePIDTunerSystem(this, new MechanumDrive.Params("","","",""),new PIDController(0,0,0),45, AngleUnit.DEGREES);
    }
}
