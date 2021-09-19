package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;

public class PIDDrive extends CommandBase {

    private final DriveBase driveBase;
    private double setPoint;
    private PIDController rightBack;
    private PIDController rightFront;
    private PIDController leftBack;
    private PIDController leftFront;
    private final double KP = 1;
    private final double KI = 0;
    private final double KD = 0;
    private final double TOLERANCE = 5;

    public PIDDrive(DriveBase system) {
        this.driveBase = system;
        addRequirements(driveBase);
        this.setPoint = 0;
        rightBack = new PIDController(KP, KI, KD);
        rightFront = new PIDController(KP, KI, KD);
        leftBack = new PIDController(KP, KI, KD);
        leftFront = new PIDController(KP, KI, KD);
    }

    @Override
    public void initialize() {
        rightBack.disableContinuousInput();
        rightBack.setTolerance(TOLERANCE);
        rightBack.reset();

        rightFront.disableContinuousInput();
        rightFront.setTolerance(TOLERANCE);
        rightFront.reset();

        leftBack.disableContinuousInput();
        leftBack.setTolerance(TOLERANCE);
        leftBack.reset();

        leftFront.disableContinuousInput();
        leftFront.setTolerance(TOLERANCE);
        leftFront.reset();
    }

    @Override
    public void execute() {
        driveBase.setRightBackMotorValue(rightBack.calculate(driveBase.getRightBackMotorEncoder(), setPoint));
        driveBase.setRightBackMotorValue(rightFront.calculate(driveBase.getRightFrontMotorEncoder(), setPoint));
        driveBase.setRightBackMotorValue(leftBack.calculate(driveBase.getLeftBackMotorEncoder(), setPoint));
        driveBase.setRightBackMotorValue(leftFront.calculate(driveBase.getLeftFrontMotorEncoder(), setPoint));
    }

    public void setSetPoint(double d) {
        this.setPoint = d;
    }

    
    
}
