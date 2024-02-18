package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeStop extends Command{
    IntakeSubsystem intake;

    public IntakeStop(IntakeSubsystem intake){
        addRequirements(intake);
        this.intake=intake;
    }

    public void initialize(){}

    public void execute(){
        intake.stop();
    }

    public boolean isFinished(){
        return false;
    }
}