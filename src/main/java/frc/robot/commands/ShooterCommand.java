package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.subsystems.Shooter;

public class ShooterCommand extends CommandBase {

    private final Shooter shoot;

    public ShooterCommand(Shooter shoot) {
        this.shoot = shoot;

        addRequirements(this.shoot);
    }

    @Override
    public void initialize(){}

    public void execute(){
        if(Robot.robotContainer.controller.getRawButton(RobotConstants.LT_AXIS)) shoot.shooterOn();
        else shoot.shooterOff();
    }

    public void end(boolean interrupted) {}

    public boolean isFinished() {return false;}
}
