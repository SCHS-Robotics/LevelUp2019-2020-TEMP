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

public class Intake extends SubSystem {

    private static final String INTAKE = "intake", OUTTAKE = "outtake";

    private DcMotor leftIntakeMotor;
    private DcMotor rightIntakeMotor;
    private DcMotor transferMotor;

    private double transferPowerMultiplier;


    private CustomizableGamepad gamepad;

    private double intakePower;

    public Intake(Robot robot, String leftIntakeConfig, String rightIntakeConfig, String transferConfig, Button intakeButton, Button outtakeButton, double intakePower, double transferPowerMultiplier) {
        super(robot);

        gamepad = new CustomizableGamepad(robot);

        this.intakePower = Math.abs(intakePower);

        leftIntakeMotor = robot.hardwareMap.dcMotor.get(leftIntakeConfig);
        rightIntakeMotor = robot.hardwareMap.dcMotor.get(rightIntakeConfig);
        transferMotor = robot.hardwareMap.dcMotor.get(transferConfig);


        transferMotor.setDirection(DcMotor.Direction.REVERSE);

        leftIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        gamepad.addButton(INTAKE, intakeButton);
        gamepad.addButton(OUTTAKE, outtakeButton);

        this.transferPowerMultiplier = transferPowerMultiplier;
    }

    public Intake(Robot robot, String leftIntakeConfig, String rightIntakeConfig, String transferConfig) {
        super(robot);

        gamepad = new CustomizableGamepad(robot);

        usesConfig = true;

        leftIntakeMotor = robot.hardwareMap.dcMotor.get(leftIntakeConfig);
        rightIntakeMotor = robot.hardwareMap.dcMotor.get(rightIntakeConfig);
        transferMotor = robot.hardwareMap.dcMotor.get(transferConfig);


        transferMotor.setDirection(DcMotor.Direction.REVERSE);

        leftIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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
            transferPowerMultiplier = data.getData("TransferPowerMultiplier", Double.class);
        }
    }

    @Override
    public void handle() {
        boolean intakeBool = gamepad.getBooleanInput(INTAKE);
        boolean outtakeBool = gamepad.getBooleanInput(OUTTAKE);

        intake(intakeBool ? intakePower : outtakeBool ? -intakePower : 0);
    }

    @Override
    public void stop() {
        stopIntake();
    }

    public void intake(double power) {
        leftIntakeMotor.setPower(power);
        rightIntakeMotor.setPower(power);
        transferMotor.setPower(transferPowerMultiplier*power);

    }

    public void intakeTime(double power, long time) {
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
                new ConfigParam(INTAKE, Button.BooleanInputs.bool_left_trigger),
                new ConfigParam(OUTTAKE, Button.BooleanInputs.bool_right_trigger),
                new ConfigParam("IntakePower",ConfigParam.numberMap(0,1,0.1),1.0),
                new ConfigParam("TransferPowerMultiplier", ConfigParam.numberMap(0,1,0.1),1.0)
        };
    }

    @AutonomousConfig
    public static ConfigParam[] autonomousConfig() {
        return new ConfigParam[] {
                new ConfigParam("IntakePower",ConfigParam.numberMap(0,1,0.1),1.0),
                new ConfigParam("TransferPowerMultiplier", ConfigParam.numberMap(0,1,0.1),1.0)
        };
    }
}
