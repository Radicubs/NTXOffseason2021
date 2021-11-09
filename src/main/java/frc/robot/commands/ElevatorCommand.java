package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.subsystems.Elevator;

public class ElevatorCommand extends CommandBase {

    private final Elevator el;

    public ElevatorCommand(Elevator el) {
        this.el = el;
        addRequirements(el);
    }

    @Override
    public void initialize(){}

    public void execute(){
        if(Robot.robotContainer.controller.getRawButton(RobotConstants.B_BUTTON)) el.elevatorOn();
        else el.elevatorOff();
    }

    public void end(boolean interrupted) {}

    public boolean isFinished() {return false;}
}
