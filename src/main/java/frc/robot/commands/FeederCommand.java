package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.subsystems.Feeder;

public class FeederCommand extends CommandBase {

    private final Feeder f;

    public FeederCommand(Feeder f) {
        this.f = f;

        addRequirements(f);
    }

    @Override
    public void initialize(){}

    public void execute(){
        if(Robot.robotContainer.controller.getRawButton(RobotConstants.Y_BUTTON)) f.feederOn();
        else f.feederOff();
    }

    public void end(boolean interrupted) {}

    public boolean isFinished() {return false;}
}
