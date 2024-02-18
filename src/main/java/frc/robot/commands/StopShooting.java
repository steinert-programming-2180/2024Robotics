package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class StopShooting extends Command{
    ShooterSubsystem shooter;

    public StopShooting(ShooterSubsystem shooter){
        addRequirements(shooter);
        this.shooter=shooter;
    }

    public void initialize(){}

    public void execute(){
        shooter.shooter_stop();
    }

    public boolean isFinished(){
        return false;
    }
}
