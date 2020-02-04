package org.firstinspires.ftc.teamcode.maincode;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.util.misc.Button;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class OpenCVTestRobot extends Robot {

    //public BLUEFoundationDetection detector;
    public REDFoundationDetection detector;
    public OpenCVTestRobot(OpMode opMode) {
        super(opMode);
        enableViewport(new Button(1, Button.BooleanInputs.noButton));

        //detector = new BLUEFoundationDetection(this);
        detector = new REDFoundationDetection(this);
    }
}