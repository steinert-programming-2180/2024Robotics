package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeForward extends Command{
    IntakeSubsystem intake;

    public IntakeForward(IntakeSubsystem intake){
        addRequirements(intake);
        this.intake=intake;
    }

    public void initialize(){}

    public void execute(){
        intake.forward();
    }

    public boolean isFinished(){
        return false;
    }
}