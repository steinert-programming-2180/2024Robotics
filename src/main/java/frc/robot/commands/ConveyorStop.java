package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorSubsystem;

public class ConveyorStop extends Command{
    ConveyorSubsystem conveyor;

    public ConveyorStop(ConveyorSubsystem conveyor){
        addRequirements(conveyor);
        this.conveyor=conveyor;
    }

    public void initialize(){}

    public void execute(){
        conveyor.stop();
    }

    public boolean isFinished(){
        return false;
    }
}