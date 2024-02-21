package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimbBackward extends Command {
Climber climber;

    public ClimbBackward(Climber climber){
        addRequirements(climber);
        this.climber=climber;
    }

    public void initialize(){}

    public void execute(){
        climber.backward();
    }
    
    public boolean isFinished(){
        return false;
    }
}
