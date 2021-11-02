package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;

public class Shooter extends SubsystemBase {
  private TalonFX shooterMotorOne;
  private TalonFX shooterMotorTwo;

  private double speed = 11500;
  // private double speed = 0.1;

  public Shooter() {

    // constructor
    shooterMotorOne = new TalonFX(RobotConstants.SHOOTER_LEFT);
    shooterMotorTwo = new TalonFX(RobotConstants.SHOOTER_RIGHT);

    shooterMotorOne.configFactoryDefault();
    shooterMotorTwo.configFactoryDefault();

    shooterMotorOne.setNeutralMode(NeutralMode.Coast);
    shooterMotorTwo.setNeutralMode(NeutralMode.Coast);

    shooterMotorOne.setInverted(true);
    shooterMotorOne.setSensorPhase(false);
    shooterMotorTwo.follow(shooterMotorOne);
    shooterMotorOne.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0);

    shooterMotorOne.config_kP(0, .05);
    shooterMotorOne.config_kF(0, 0.0499999523);
    shooterMotorOne.config_kD(0, .9);
  }

  public void shootBall(double speed) {
    // shooterMotorOne.set(ControlMode.PercentOutput, speed);
    // shooterMotorOne.set(ControlMode.Velocity, -11500);
    shooterMotorOne.set(ControlMode.Velocity, speed);
    System.out.println("meow");
    System.out.println(shooterMotorOne.getSelectedSensorVelocity());
    System.out.println(shooterMotorOne.getMotorOutputPercent());
    // shooterMotorTwo.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    shooterMotorOne.set(ControlMode.PercentOutput, -0.5);
    shooterMotorTwo.set(ControlMode.PercentOutput, -0.5);
  }

  public void shooterOn() {
    
    speed = 0.25;
  }

  public void shooterOff() {
    
    speed = 0;
  }
  //@Override
  //public void initDefaultCommand() {
  //  setDefaultCommand(new RunShooter(speed));
  //}
}
