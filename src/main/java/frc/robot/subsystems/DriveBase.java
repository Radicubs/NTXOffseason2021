// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.RobotConstants;
import frc.robot.commands.MecanumDriveControl;
import frc.robot.commands.PIDDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class DriveBase extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  // Right Motors
  private WPI_TalonFX rightMotorFront;
  private WPI_TalonFX rightMotorBack;

  // Left Motors
  private WPI_TalonFX leftMotorFront;
  private WPI_TalonFX leftMotorBack;

  private MecanumDrive m_robotDrive;

  public DriveBase() {
    //Redone for PID
    // motors
    rightMotorFront = new WPI_TalonFX(RobotConstants.RIGHT_FALCON_FRONT);
    rightMotorBack = new WPI_TalonFX(RobotConstants.RIGHT_FALCON_BACK);

    leftMotorFront = new WPI_TalonFX(RobotConstants.LEFT_FALCON_FRONT);
    leftMotorBack = new WPI_TalonFX(RobotConstants.LEFT_FALCON_BACK);
    m_robotDrive = new MecanumDrive(leftMotorFront, leftMotorBack, rightMotorFront, rightMotorBack);
    rightMotorFront.configFactoryDefault();
    rightMotorBack.configFactoryDefault();
    leftMotorFront.configFactoryDefault();
    leftMotorBack.configFactoryDefault();

    leftMotorBack.getSelectedSensorVelocity();

    // Might interfere with PID

    leftMotorFront.setNeutralMode(NeutralMode.Brake);
    leftMotorBack.setNeutralMode(NeutralMode.Brake);
    rightMotorFront.setNeutralMode(NeutralMode.Brake);
    rightMotorBack.setNeutralMode(NeutralMode.Brake);

    PIDDrive drive = new PIDDrive(this);
    drive.setSetPoint(10);

    setDefaultCommand(drive);
  }

  @Override
  public void periodic() {}

  public void setValues(double m1, double m2, double m3, double m4) {
    rightMotorBack.set(ControlMode.PercentOutput, m1);
    rightMotorFront.set(ControlMode.PercentOutput, m2);
    leftMotorBack.set(ControlMode.PercentOutput, m3);
    leftMotorFront.set(ControlMode.PercentOutput, m4);
  
  }

  public void setRightBackMotorValue(double speed) {
    rightMotorBack.set(ControlMode.PercentOutput, speed);
  }

  public void setRightFrontMotorValue(double speed) {
    rightMotorFront.set(ControlMode.PercentOutput, speed);
  }

  public void setLeftBackMotorValue(double speed) {
    leftMotorBack.set(ControlMode.PercentOutput, speed);
  }

  public void setLeftFrontMotorValue(double speed) {
    leftMotorFront.set(ControlMode.PercentOutput, speed);
  }

  public double getRightBackMotorEncoder() {
    return rightMotorBack.getSelectedSensorPosition();
  }

  public double getRightFrontMotorEncoder() {
    return rightMotorFront.getSelectedSensorPosition();
  }

  public double getLeftBackMotorEncoder() {
    return leftMotorBack.getSelectedSensorPosition();
  }

  public double getLeftFrontMotorEncoder() {
    return leftMotorFront.getSelectedSensorPosition();
  }

public void setValues(double ySpeed, double xSpeed, double zRotation)
{
  m_robotDrive.driveCartesian(ySpeed, xSpeed, zRotation);
}

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public double[] getEncoderValues(){
    double[] vals = {rightMotorBack.getSelectedSensorPosition(), rightMotorFront.getSelectedSensorPosition(), leftMotorBack.getSelectedSensorPosition(), leftMotorFront.getSelectedSensorPosition(),
    };
    return vals;
  }

  
}
