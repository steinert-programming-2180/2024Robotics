package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorSubsystem;

public class StoppedConveyorForward extends Command{
    ConveyorSubsystem conveyor;

    public StoppedConveyorForward(ConveyorSubsystem conveyor){
        addRequirements(conveyor);
        this.conveyor=conveyor;
    }
    @Override
    public void initialize(){
        
    }

    public void execute(){
        conveyor.forward();
    }

    public boolean isFinished(){
        return conveyor.hasNote();
    }

    @Override
    public void end(boolean inturrept) {
        conveyor.stop();
        conveyor.backwords();
        Timer.delay(0.05);
        conveyor.stop();
    }
}