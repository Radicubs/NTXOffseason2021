// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.RobotConstants;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class DriveBase extends SubsystemBase implements Subsystem {
  /** Creates a new ExampleSubsystem. */

  // Right Motors
  private TalonFX rightMotorFront;
  private TalonFX rightMotorBack;

  // Left Motors
  private TalonFX leftMotorFront;
  private TalonFX leftMotorBack;

  public DriveBase() {
    
    
    // motors
    rightMotorFront = new TalonFX(RobotConstants.RIGHT_FALCON_FRONT);
    rightMotorBack = new TalonFX(RobotConstants.RIGHT_FALCON_BACK);

    leftMotorFront = new TalonFX(RobotConstants.LEFT_FALCON_FRONT);
    leftMotorBack = new TalonFX(RobotConstants.LEFT_FALCON_BACK);

    rightMotorFront.configFactoryDefault();
    rightMotorBack.configFactoryDefault();
    leftMotorFront.configFactoryDefault();
    leftMotorBack.configFactoryDefault();

    // Might interfere with PID

    leftMotorFront.setNeutralMode(NeutralMode.Brake);
    leftMotorBack.setNeutralMode(NeutralMode.Brake);
    rightMotorFront.setNeutralMode(NeutralMode.Brake);
    rightMotorBack.setNeutralMode(NeutralMode.Brake);

  }

  @Override
  public void periodic() {}

  public void setValues(double m1, double m2, double m3, double m4) {
    rightMotorBack.set(ControlMode.PercentOutput, m1);
    rightMotorFront.set(ControlMode.PercentOutput, m2);
    leftMotorBack.set(ControlMode.PercentOutput, m3);
    leftMotorFront.set(ControlMode.PercentOutput, m4);
  
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  
}
