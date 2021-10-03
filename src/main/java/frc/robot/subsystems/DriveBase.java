// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//package frc.robot.subsystems;
package frc.robot.subsystems;
import frc.robot.RobotConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.MecanumDriveControl;
//import frc.robot.commands.PIDDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveWheelSpeeds;

public class DriveBase extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  // Right Motors
  private WPI_TalonFX rightMotorFront;
  private WPI_TalonFX rightMotorBack;

  // Left Motors
  private WPI_TalonFX leftMotorFront;
  private WPI_TalonFX leftMotorBack;

  private MecanumDrive m_robotDrive;

  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);
  MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics(
    m_frontLeftLocation,
    m_frontRightLocation,
    m_backLeftLocation,
    m_backRightLocation
  );

  MecanumDriveOdometry m_odometry;

  PIDController frontLeftPID = new PIDController(RobotConstants.fl_kP, 0, 0);
  PIDController frontRightPID = new PIDController(RobotConstants.fr_kP, 0, 0);
  PIDController backLeftPID = new PIDController(RobotConstants.bl_kP, 0, 0);
  PIDController backRightPID = new PIDController(RobotConstants.br_kP, 0, 0);

  AHRS m_Gyro = RobotContainer.ahrs;

  Pose2d pose = new Pose2d();

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(RobotConstants.kS, RobotConstants.kV, RobotConstants.kA);

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

    // Might interfere with PID

    leftMotorFront.setNeutralMode(NeutralMode.Brake);
    leftMotorBack.setNeutralMode(NeutralMode.Brake);
    rightMotorFront.setNeutralMode(NeutralMode.Brake);
    rightMotorBack.setNeutralMode(NeutralMode.Brake);

    MecanumDriveControl drive = new MecanumDriveControl(this);
    //drive.setSetPoint(10);

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
  var mecanumDriveWheelSpeeds = m_kinematics.toWheelSpeeds(
        	new ChassisSpeeds(ySpeed, xSpeed, zRotation)
		);
    	mecanumDriveWheelSpeeds.normalize(1);
		setSpeeds(mecanumDriveWheelSpeeds);
		
  	}
  //m_robotDrive.driveCartesian(ySpeed, xSpeed, zRotation);


public MecanumDriveWheelSpeeds getSpeeds() { 
  return new MecanumDriveWheelSpeeds(
      ((leftMotorFront.getSelectedSensorVelocity() * 10 * 2*RobotConstants.kWheelRadius * Math.PI) / (2048 * RobotConstants.kGearRatio)),
      ((rightMotorFront.getSelectedSensorVelocity() * 10 * 2*RobotConstants.kWheelRadius * Math.PI) / (2048 * RobotConstants.kGearRatio)),
      ((leftMotorBack.getSelectedSensorVelocity() * 10 * 2*RobotConstants.kWheelRadius * Math.PI) / (2048 * RobotConstants.kGearRatio)),
      ((rightMotorBack.getSelectedSensorVelocity() * 10 * 2*RobotConstants.kWheelRadius * Math.PI) / (2048 * RobotConstants.kGearRatio))
  );
}

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public double[] getEncoderValues(){
    double[] vals = {rightMotorBack.getSelectedSensorPosition(), rightMotorFront.getSelectedSensorPosition(), leftMotorBack.getSelectedSensorPosition(), leftMotorFront.getSelectedSensorPosition(),};
    return vals;
  }

  public PIDController getFrontLeftPIDController() {
      return frontLeftPID;
  }

  public PIDController getFrontRightPIDController() {
      return frontRightPID;
  }

  public PIDController getBackLeftPIDController() {
      return backLeftPID;
  }

  public PIDController getBackRightPidController() {
      return backRightPID;
  }

  public void resetEncoders() {
    leftMotorFront.setSelectedSensorPosition(0);
    leftMotorBack.setSelectedSensorPosition(0);
    rightMotorFront.setSelectedSensorPosition(0);
    rightMotorBack.setSelectedSensorPosition(0);
  }

  public void resetOdometry(Pose2d newPose) {
    resetEncoders();
    m_odometry.resetPosition(newPose, getHeading());
  }

  public void initializeOdometry() {
    m_odometry = new MecanumDriveOdometry(m_kinematics, getHeading());
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(getGyroHeading());
  }

  private double getGyroHeading() {
    return Math.IEEEremainder(m_Gyro.getAngle(), 360) * -1;
  } 

  public Pose2d getPose() {
    return pose;
  }

  public MecanumDriveKinematics getKinematics() {
    return m_kinematics;
  }

  public void setSpeeds(MecanumDriveWheelSpeeds speeds) {
    leftMotorFront.selectProfileSlot(0, 0);
    rightMotorFront.selectProfileSlot(0, 0);
    leftMotorBack.selectProfileSlot(0, 0);
    rightMotorBack.selectProfileSlot(0, 0);

    ControlMode v = ControlMode.Velocity;
    double n = 2048 / 10 / (2*Math.PI*RobotConstants.kWheelRadius);
    double setLFVelocity = speeds.frontLeftMetersPerSecond * n;
		double setRFVelocity = speeds.frontRightMetersPerSecond * n;
		double setLRVelocity = speeds.rearLeftMetersPerSecond * n;
		double setRRVelocity = speeds.rearRightMetersPerSecond * n;
    System.out.println(setLFVelocity);
		leftMotorFront.set(v, setLFVelocity);
		rightMotorFront.set(v, setRFVelocity);
		leftMotorBack.set(v, setLRVelocity);
		rightMotorBack.set(v, setRRVelocity);
  }
}
