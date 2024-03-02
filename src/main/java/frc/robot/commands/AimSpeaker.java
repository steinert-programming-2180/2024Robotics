package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AimSpeaker extends Command{
    ShooterSubsystem shooter;
    LimelightSubsystem llight;

    public AimSpeaker(ShooterSubsystem shooter, LimelightSubsystem llight){
        addRequirements(shooter, llight);
        this.llight = llight;
        this.shooter = shooter;
    }

    public void initialize(){}

    public void execute(){
        double angle = this.llight.getDistanceToSpeaker();
        if (angle == 0) return;

        shooter.setAngle(angle);
    }
    
    public boolean isFinished(){
        return false;
    }
}
