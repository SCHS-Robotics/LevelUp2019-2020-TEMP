package org.firstinspires.ftc.teamcode.maincode;

import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.system.subsystems.MechanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class Cygnus extends Robot {

    public MechanumDrive drive;
    public Intake intake;
    public LineDetector lineDetector;
    public CapstonePlopper plopper;
    public FoundationMover mover;
    public Hugger hugger;
   // public YeetAutonomousProgram auto;
    //public Intake intake;
    public Cygnus(OpMode opMode) {
        super(opMode);

        //enableViewport(new Button(1, Button.BooleanInputs.noButton));

        drive = new MechanumDrive(this,"topLeft","topRight","botLeft","botRight");
        intake = new Intake(this, "intakeLeft", "intakeRight", "pull");
        lineDetector = new LineDetector(this, "colorSensor");
        plopper = new CapstonePlopper(this,"plop");
        mover = new FoundationMover(this,"leftLatch", "rightLatch");
        hugger = new Hugger(this, "hugger");
        // auto = new YeetAutonomousProgram();

        //("Autonomous", auto);
    }
}
