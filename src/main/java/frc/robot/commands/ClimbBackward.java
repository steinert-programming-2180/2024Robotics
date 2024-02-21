package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ConveyorSubsystem;

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
