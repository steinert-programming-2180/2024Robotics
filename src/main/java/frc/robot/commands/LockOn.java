package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.limelightConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class LockOn extends Command {
    private DriveSubsystem drive;
    private LimelightSubsystem llight;
    private ShooterSubsystem shooter;
    Timer timeout = new Timer();

    public LockOn(DriveSubsystem drive, LimelightSubsystem llight, ShooterSubsystem shooter) {
        addRequirements(drive, llight, shooter);
        this.drive = drive;
        this.llight = llight;
        this.shooter = shooter;
    }

    public void execute() {
        timeout.start();
        shooter.setAngle(limelightConstants.calculateShooterAngle(llight.getDistanceToSpeaker()));
        drive.drive(0, 0, MathUtil.clamp(llight.getTx() * ShooterConstants.lockOnP, -.3, .3), true, true);
    }

    @Override
    public void end (boolean force) {
        drive.drive(0, 0, 0, true, true);
        timeout.stop();
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(llight.getTx()) <= .1) || timeout.get() >= 1.5;
    }
}
