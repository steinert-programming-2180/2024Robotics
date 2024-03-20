package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.limelightConstants;
import frc.robot.Constants.DriveConstants.GyroOffset;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class LockOnArmAuto extends Command {
    private LimelightSubsystem llight;
    private ShooterSubsystem shooter;
    Timer timeout = new Timer();

    public LockOnArmAuto(LimelightSubsystem llight, ShooterSubsystem shooter) {
        addRequirements(llight, shooter);
        this.llight = llight;
        this.shooter = shooter;
    }

    public void execute() {
        timeout.start();
        shooter.setAngle(limelightConstants.calculateShooterAngle(llight.getDistanceToSpeaker()));
        // shooter.setAngle(limelightConstants.calculateShooterAngle(llight.getDistanceToSpeaker()));
        
    }

    @Override
    public void end (boolean force) {
        // drive.drive(0, 0, 0, true, true);
        timeout.stop();
    }

    @Override
    public boolean isFinished() {
        // || timeout.get() >= 1.5
        return true;
        //  && timeout.get() >= 1.0;
    }
}
