// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.subsystems.DriveBase;

/** An example command that uses an example subsystem. */
public class TankDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveBase driveBase;
  private PIDController leftBack = new PIDController(0, 0, 0);
  private PIDController leftFront = new PIDController(0, 0, 0);
  private PIDController rightBack = new PIDController(0, 0, 0);
  private PIDController rightFront = new PIDController(0, 0, 0);

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TankDrive(DriveBase subsystem) {
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

    double left = Robot.robotContainer.controller.getRawAxis(RobotConstants.LEFT_Y_AXIS) / 10;
    double right = Robot.robotContainer.controller.getRawAxis(RobotConstants.RIGHT_Y_AXIS) / 10;

    

    driveBase.setValues(rightBack.calculate(driveBase.getEncoderValues()[0], 180), rightFront.calculate(driveBase.getEncoderValues()[1], 180)
      , leftBack.calculate(driveBase.getEncoderValues()[2], 180), leftFront.calculate(driveBase.getEncoderValues()[3], 180));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
