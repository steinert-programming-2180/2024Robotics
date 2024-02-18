package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ShooterSubsystem;

public class AimAmp extends Command{
    ShooterSubsystem shooter;

    public AimAmp(ShooterSubsystem shooter){
        addRequirements(shooter);
        this.shooter=shooter;
    }

    public void initialize(){}

    public void execute(){
    }
    
    public boolean isFinished(){
        return false;
    }
}
