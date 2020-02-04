package org.firstinspires.ftc.teamcode.maincode;


import com.SCHSRobotics.HAL9001.system.source.BaseRobot.Robot;
import com.SCHSRobotics.HAL9001.system.source.BaseRobot.SubSystem;
import com.SCHSRobotics.HAL9001.util.annotations.AutonomousConfig;
import com.SCHSRobotics.HAL9001.util.annotations.TeleopConfig;
import com.SCHSRobotics.HAL9001.util.misc.Button;
import com.SCHSRobotics.HAL9001.util.misc.ConfigData;
import com.SCHSRobotics.HAL9001.util.misc.ConfigParam;
import com.SCHSRobotics.HAL9001.util.misc.CustomizableGamepad;
import com.qualcomm.robotcore.hardware.DcMotor;

public class StackerIntake extends SubSystem {

    private static final String INTAKE = "stackIntake", OUTTAKE = "stackOuttake";

    private DcMotor leftIntakeMotor;
    private DcMotor rightIntakeMotor;

    private CustomizableGamepad gamepad;

    private double intakePower;

    public StackerIntake(Robot robot, String leftIntakeConfig, Button intakeButton, Button outtakeButton, double intakePower) {
        super(robot);

        gamepad = new CustomizableGamepad(robot);

        this.intakePower = Math.abs(intakePower);

        leftIntakeMotor = robot.hardwareMap.dcMotor.get(leftIntakeConfig);
        //rightIntakeMotor = robot.hardwareMap.dcMotor.get(rightIntakeConfig);

        gamepad.addButton(INTAKE, intakeButton);
        gamepad.addButton(OUTTAKE, outtakeButton);
    }

    public StackerIntake(Robot robot, String leftIntakeConfig) {
        super(robot);

        gamepad = new CustomizableGamepad(robot);

        usesConfig = true;

        leftIntakeMotor = robot.hardwareMap.dcMotor.get(leftIntakeConfig);
        //rightIntakeMotor = robot.hardwareMap.dcMotor.get(rightIntakeConfig);

        leftIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rightIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void init() {

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        if (usesConfig) {
            gamepad = robot.pullControls(this);
            ConfigData data = robot.pullNonGamepad(this);
            intakePower = data.getData("IntakePower", Double.class);
        }
    }

    @Override
    public void handle() {
        boolean intake = gamepad.getBooleanInput(INTAKE);
        boolean outtake = gamepad.getBooleanInput(OUTTAKE);

        intake(intake ? intakePower : outtake ? -intakePower : 0);
    }

    @Override
    public void stop() {
        stopIntake();
    }

    public void intake(double power) {
        leftIntakeMotor.setPower(power);
        rightIntakeMotor.setPower(power);
    }

    public void intakeTime(double power, long time) {
        long startTime = System.currentTimeMillis();
        intake(power);
        waitTime(time);
        stopIntake();
    }

    public void stopIntake() {
        intake(0);
    }

    @TeleopConfig
    public static ConfigParam[] teleopConfig() {
        return new ConfigParam[] {
                new ConfigParam(INTAKE, Button.BooleanInputs.right_bumper),
                new ConfigParam(OUTTAKE, Button.BooleanInputs.left_bumper),
                new ConfigParam("IntakePower",ConfigParam.numberMap(0,1,0.1),1.0)
        };
    }

    @AutonomousConfig
    public static ConfigParam[] autonomousConfig() {
        return new ConfigParam[] {
                new ConfigParam("IntakePower",ConfigParam.numberMap(0,1,0.1),1.0)
        };
    }
}
