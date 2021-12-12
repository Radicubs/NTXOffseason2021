// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.drive.Vector2d;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.subsystems.DriveBase;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class MecanumDriveControl extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DriveBase driveBase;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public MecanumDriveControl(DriveBase subsystem) {
        driveBase = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        double left = Robot.robotContainer.controller.getRawAxis(RobotConstants.LEFT_Y_AXIS) / 3;
        double right = Robot.robotContainer.controller.getRawAxis(RobotConstants.LEFT_X_AXIS) / 3;
        double zRot = Robot.robotContainer.controller.getRawAxis(RobotConstants.RIGHT_X_AXIS) / 3;

        WheelSpeeds speeds = driveCartesianIK(applyDeadband(left, 0.02), applyDeadband(right, 0.02), zRot, 0);

        driveBase.setValues(speeds.rearRight, speeds.frontRight, speeds.rearLeft, speeds.frontLeft);


    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }




    public static double applyDeadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    public WheelSpeeds driveCartesianIK(
            double ySpeed, double xSpeed, double zRotation, double gyroAngle) {

        if(ySpeed < -1) ySpeed = -1;
        else if(ySpeed > 1) ySpeed = 1;

        if(xSpeed < -1) xSpeed = -1;
        else if(xSpeed > 1) xSpeed = 1;

        // Compensate for gyro angle.
        Vector2d input = new Vector2d(ySpeed, xSpeed);
        input.rotate(-gyroAngle);

        double[] wheelSpeeds = new double[4];
        wheelSpeeds[RobotDriveBase.MotorType.kFrontLeft.value] = input.x + input.y + zRotation;
        wheelSpeeds[RobotDriveBase.MotorType.kFrontRight.value] = input.x - input.y - zRotation;
        wheelSpeeds[RobotDriveBase.MotorType.kRearLeft.value] = input.x - input.y + zRotation;
        wheelSpeeds[RobotDriveBase.MotorType.kRearRight.value] = input.x + input.y - zRotation;

        normalize(wheelSpeeds);

        return new WheelSpeeds(
                wheelSpeeds[RobotDriveBase.MotorType.kFrontLeft.value],
                wheelSpeeds[RobotDriveBase.MotorType.kFrontRight.value],
                wheelSpeeds[RobotDriveBase.MotorType.kRearLeft.value],
                wheelSpeeds[RobotDriveBase.MotorType.kRearRight.value]);
    }

    protected static void normalize(double[] wheelSpeeds) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        for (int i = 1; i < 4; i++) {
            double temp = Math.abs(wheelSpeeds[i]);
            if (maxMagnitude < temp) {
                maxMagnitude = temp;
            }
        }
        if (maxMagnitude > 1.0) {
            for (int i = 0; i < 4; i++) {
                wheelSpeeds[i] = wheelSpeeds[i] / maxMagnitude;
            }
        }
    }

    public static class WheelSpeeds {
        public double frontLeft;
        public double frontRight;
        public double rearLeft;
        public double rearRight;

        /** Constructs a WheelSpeeds with zeroes for all four speeds. */
        public WheelSpeeds() {}

        /**
         * Constructs a WheelSpeeds.
         *
         * @param frontLeft The front left speed.
         * @param frontRight The front right speed.
         * @param rearLeft The rear left speed.
         * @param rearRight The rear right speed.
         */
        public WheelSpeeds(double frontLeft, double frontRight, double rearLeft, double rearRight) {
            this.frontLeft = frontLeft;
            this.frontRight = frontRight;
            this.rearLeft = rearLeft;
            this.rearRight = rearRight;
        }
    }

}