package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class LockOn extends Command {
    private DriveSubsystem drive;
    private LimelightSubsystem llight;

    public LockOn(DriveSubsystem drive, LimelightSubsystem llight) {
        addRequirements(drive, llight);
        this.drive = drive;
        this.llight = llight;
    }

    public void execute() {
           drive.drive(0, 0, MathUtil.clamp(llight.getTx() * -0.01, -.5, .5), true, true);
    }

    @Override
    public void end (boolean force) {
        drive.drive(0, 0, 0, true, true);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(llight.getTx()) <= .5;
    }
}
