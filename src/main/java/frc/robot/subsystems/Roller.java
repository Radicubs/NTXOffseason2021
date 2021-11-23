package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.commands.ElevatorCommand;

public class Roller extends SubsystemBase {
  private CANSparkMax roller;

  private double speed = 0.0;

  public Roller() {
    roller = new CANSparkMax(RobotConstants.ROLLER, MotorType.kBrushless);
 
    // Set Motors to default and neutral
    roller.restoreFactoryDefaults();

    roller.setSmartCurrentLimit(40);
    roller.setSecondaryCurrentLimit(40);
  }

  public void rollerOn() {
    speed = 0.5;
  }
  public void rollerOff() {
    speed = 0;
  }

  @Override
  public void periodic() {
    roller.set(-speed);
  }

  //@Override
  //public void initDefaultCommand() {
  //  setDefaultCommand(new RunElevator(speed));
  //}
}
