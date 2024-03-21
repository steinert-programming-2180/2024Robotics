package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeForwardAuto extends Command{
    IntakeSubsystem intake;

    public IntakeForwardAuto(IntakeSubsystem intake){
        addRequirements(intake);
        this.intake=intake;
    }

    public void initialize(){}

    public void execute(){
        intake.forward();
    }

    public void end() {
        // Autooooooooooooooooooo
    }

    public boolean isFinished(){
        return true;
    }
}