package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.commands.ElevatorCommand;

public class Elevator extends SubsystemBase {
  private CANSparkMax elevatorBack;
  private CANSparkMax elevatorFront;

  private double speed = 0;

  public Elevator() {
    elevatorBack = new CANSparkMax(RobotConstants.ELEVATOR_BACK, MotorType.kBrushless);
    elevatorFront = new CANSparkMax(RobotConstants.ELEVATOR_FRONT, MotorType.kBrushless);

    // Set Motors to default and neutral
    elevatorBack.restoreFactoryDefaults();
    elevatorFront.restoreFactoryDefaults();

    //elevatorBack.setSmartCurrentLimit(10);
    //elevatorBack.setSecondaryCurrentLimit(10);
    elevatorFront.setSmartCurrentLimit(20);
    elevatorFront.setSecondaryCurrentLimit(20);
  }


  @Override
  public void periodic() {
    elevatorBack.set(-speed);
    elevatorFront.set(speed);

  }

  public void elevatorOn() {
    
    speed = 0.5;
  }

  public void elevatorOff() {
    
    speed = 0;
  }
  //@Override
  //public void initDefaultCommand() {
  //  setDefaultCommand(new RunElevator(speed));
  //}
}
