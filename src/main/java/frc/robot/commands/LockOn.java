package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
        shooter.setAngle(limelightConstants.calculateShooterAngleQuad(llight.getDistanceToSpeaker()));
        if (llight.getTx() == 0) {
            end(true);
        } else {
            drive.drive(0, 0, MathUtil.clamp(llight.getTx() * ShooterConstants.lockOnP, -.5, .5), true, true);
        }
    }

    @Override
    public void end (boolean force) {
        drive.drive(0, 0, 0, true, true);
        timeout.stop();
    }

    @Override
    public boolean isFinished() {
        // || timeout.get() >= 1.5
        return (Math.abs(llight.getTx()) <= .05) || timeout.get() >= 1;
    }
}
