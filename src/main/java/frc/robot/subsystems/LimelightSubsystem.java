package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
    NetworkTable limeLightTable;

    public LimelightSubsystem() {
        limeLightTable = NetworkTableInstance.getDefault().getTable("limelight");
    }
}
