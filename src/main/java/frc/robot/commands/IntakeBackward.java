package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeBackward extends Command{
    IntakeSubsystem intake;

    public IntakeBackward(IntakeSubsystem intake){
        addRequirements(intake);
        this.intake=intake;
    }

    public void initialize(){}

    public void execute(){
        intake.backwords();
    }

    public boolean isFinished(){
        return false;
    }
}