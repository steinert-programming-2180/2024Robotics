package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimbForward extends Command {
Climber climber;

    public ClimbForward(Climber climber){
        addRequirements(climber);
        this.climber=climber;
    }

    public void initialize(){}

    public void execute(){
        climber.forward();
    }
    
    public boolean isFinished(){
        return false;
    }
}
