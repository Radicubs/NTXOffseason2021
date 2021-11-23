package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.commands.FeederCommand;

public class Feeder extends SubsystemBase {
  private CANSparkMax feederMotorOne;
  private CANSparkMax feederMotorTwo;

  private double speed = 0;

  public Feeder() {

    // constructor
    feederMotorOne = new CANSparkMax(RobotConstants.FEEDER_LEFT, MotorType.kBrushless);
    feederMotorTwo = new CANSparkMax(RobotConstants.FEEDER_RIGHT, MotorType.kBrushless);

    // Set Motors to default and neutral
    feederMotorOne.restoreFactoryDefaults();
    feederMotorTwo.restoreFactoryDefaults();

    feederMotorOne.setSmartCurrentLimit(5);
    feederMotorOne.setSecondaryCurrentLimit(5);
    feederMotorTwo.setSmartCurrentLimit(5);
    feederMotorTwo.setSecondaryCurrentLimit(5);
  }

  @Override
  public void periodic() {
    feederMotorOne.set(speed);
    feederMotorTwo.set(-speed);
  }

  public void feederOn() {
    
    speed = 0.25;
  }

  public void feederOff() {
    
    speed = 0;
  }

  //@Override
  //public void initDefaultCommand() {
  //  setDefaultCommand(new RunFeeder(speed));
  //}
}
