package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class RaiseShooter extends Command{
    ShooterSubsystem shooter;

    public RaiseShooter(ShooterSubsystem shooter){
        addRequirements(shooter);
        this.shooter=shooter;
    }

    public void initialize(){}

    public void execute(){
        shooter.arm_up();
    }

    public boolean isFinished(){
        return false;
    }
}
