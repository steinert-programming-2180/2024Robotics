package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class LockOn extends Command {
    private DriveSubsystem drive;
    private LimelightSubsystem llight;
    Timer timeout = new Timer();

    public LockOn(DriveSubsystem drive, LimelightSubsystem llight) {
        addRequirements(drive, llight);
        this.drive = drive;
        this.llight = llight;
    }

    public void execute() {
        timeout.start();
        drive.drive(0, 0, MathUtil.clamp(llight.getTx() * ShooterConstants.lockOnP, -.3, .3), true, true);
    }

    @Override
    public void end (boolean force) {
        drive.drive(0, 0, 0, true, true);
        timeout.stop();
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(llight.getTx()) <= .5) || timeout.get() >= 1.5;
    }
}
