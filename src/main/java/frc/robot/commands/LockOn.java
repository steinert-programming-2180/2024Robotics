package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class LockOn extends Command {
    
    public LockOn(DriveSubsystem drive, LimelightSubsystem llight) {
        addRequirements(drive, llight);
    }

    public void execute() {
        
    }
}
