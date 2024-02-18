package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class StopShooter extends Command{
    ShooterSubsystem shooter;

    public StopShooter(ShooterSubsystem shooter){
        addRequirements(shooter);
        this.shooter=shooter;
    }

    public void initialize(){}

    public void execute(){
        shooter.arm_stop();
    }

    public boolean isFinished(){
        return false;
    }
}
