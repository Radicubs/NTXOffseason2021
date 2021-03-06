// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.TankDrive;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveBase driveBase = new DriveBase();
  private final Feeder feeder = new Feeder();
  private final Elevator elevetor = new Elevator();
  private final Elevator elevator = elevetor;
  private final Shooter shooter = new Shooter();
  private final Roller roller = new Roller();
  
  public final JoystickButton roller_button = new JoystickButton(controller, RobotConstants.A_BUTTON);
  public final JoystickButton feeder_button = new JoystickButton(controller, RobotConstants.B_BUTTON);
  public final JoystickButton elevator_button = new JoystickButton(controller, RobotConstants.Y_BUTTON);
  public final JoystickButton shooter_button = new JoystickButton(controller, RobotConstants.X_BUTTON);

  private final TankDrive m_autoCommand = new TankDrive(driveBase);
  public static Joystick controller = new Joystick(RobotConstants.JOYSTICK);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  public void configureButtonBindings() {

    feeder_button.toggleWhenPressed(new StartEndCommand(feeder::feederOn,(feeder::feederOff),(feeder)));
    elevator_button.toggleWhenPressed(new StartEndCommand(elevator::elevatorOn,(elevator::elevatorOff),(elevator)));
    shooter_button.toggleWhenPressed(new StartEndCommand(shooter::shooterOn,(shooter::shooterOff),(shooter)));
    roller_button.toggleWhenPressed(new StartEndCommand(roller::rollerOn,(roller::rollerOff),(roller)));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
