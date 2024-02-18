package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorSubsystem;

public class ConveyorForward extends Command{
    ConveyorSubsystem conveyor;

    public ConveyorForward(ConveyorSubsystem conveyor){
        addRequirements(conveyor);
        this.conveyor=conveyor;
    }

    public void initialize(){}

    public void execute(){
        conveyor.forward();
    }

    public boolean isFinished(){
        return false;
    }
}