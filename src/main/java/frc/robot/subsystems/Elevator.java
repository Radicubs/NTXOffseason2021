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

  private double speed = 0.75;

  public Elevator() {
    elevatorBack = new CANSparkMax(RobotConstants.ELEVATOR_BACK, MotorType.kBrushless);
    elevatorFront = new CANSparkMax(RobotConstants.ELEVATOR_FRONT, MotorType.kBrushless);

    // Set Motors to default and neutral
    elevatorBack.restoreFactoryDefaults();
    elevatorFront.restoreFactoryDefaults();

    elevatorBack.setSmartCurrentLimit(2);
    elevatorBack.setSecondaryCurrentLimit(2);
    elevatorFront.setSmartCurrentLimit(2);
    elevatorFront.setSecondaryCurrentLimit(2);

    setDefaultCommand(new ElevatorCommand(this));
  }

  public void elevatorUp(double speed) {
    // elevatorBack.set(-0.75);
    System.out.println("Back current: " + elevatorBack.getOutputCurrent());
    // System.out.println("Back volta∂∂∂∂ge: " +
    // elevatorBack.getVoltageCompensationNominalVoltage());

    /* 
     * if (elevatorBack.getOutputCurrent() < 0.1) { elevatorBack.set(speed); }
     */
    elevatorBack.set(speed / 2);
  }

  @Override
  public void periodic() {
    elevatorBack.set(-speed);
    elevatorFront.set(speed);

  }

  public void elevatorOn() {
    
    speed = 0.25;
  }

  public void elevatorOff() {
    
    speed = 0;
  }
  //@Override
  //public void initDefaultCommand() {
  //  setDefaultCommand(new RunElevator(speed));
  //}
}
