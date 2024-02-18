package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ConveyorSubsystem;

public class ConveyorBackward extends Command{
    ConveyorSubsystem conveyor;

    public ConveyorBackward(ConveyorSubsystem conveyor){
        addRequirements(conveyor);
        this.conveyor=conveyor;
    }

    public void initialize(){}

    public void execute(){
        conveyor.backwords();
    }
    
    public boolean isFinished(){
        return false;
    }
}
