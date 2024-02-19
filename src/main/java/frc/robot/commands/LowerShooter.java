package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class LowerShooter extends Command{
    ShooterSubsystem shooter;

    public LowerShooter(ShooterSubsystem shooter){
        addRequirements(shooter);
        this.shooter=shooter;
    }

    public void initialize(){}

    public void execute(){
        shooter.arm_down();
    }

    public boolean isFinished(){
        return false;
    }
}
