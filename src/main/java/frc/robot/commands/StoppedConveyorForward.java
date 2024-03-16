package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorSubsystem;

public class StoppedConveyorForward extends Command{
    ConveyorSubsystem conveyor;
    Spark m_blinkinSpark;

    public StoppedConveyorForward(ConveyorSubsystem conveyor){
        addRequirements(conveyor);
        this.conveyor=conveyor;
        this.m_blinkinSpark = conveyor.m_blinkinSpark;
    }
    @Override
    public void initialize(){
        
    }

    public void execute(){
        conveyor.fastForward();
    }

    public boolean isFinished(){
        if (conveyor.hasNote()) {
            conveyor.stop();
            Timer.delay(0.05);
            conveyor.backwords();
            Timer.delay(0.05);
            conveyor.flashBlinkin();
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean inturrept) {
        conveyor.stop();
        conveyor.backwords();
        Timer.delay(0.05);
        conveyor.stop();
    }
}