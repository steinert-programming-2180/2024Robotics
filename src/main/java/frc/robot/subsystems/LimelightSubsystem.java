package frc.robot.subsystems;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
    NetworkTable limeLightTable;
    NetworkTableEntry m_botPos;

    public LimelightSubsystem() {
        limeLightTable = NetworkTableInstance.getDefault().getTable("limelight");
        m_botPos = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose");
    }

    public double getBotX(){
        double[] botPose = m_botPos.getDoubleArray(new double[6]);
        return botPose[0];
    }

    public double getBotY(){
        double[] botPose = m_botPos.getDoubleArray(new double[6]);
        return botPose[1];
    }

    public double getAngle() {
        double[] botPose = m_botPos.getDoubleArray(new double[6]);
        return botPose[5];
    }

    public void periodic(){
        double[] botPose = m_botPos.getDoubleArray(new double[6]);

        if (botPose.length != 0) {
            SmartDashboard.putNumber("x bot pose", getBotX());
            SmartDashboard.putNumber("y bot pose", getBotY());

            SmartDashboard.putNumber("theta z bot pose", getBotX());
        }
    }
}
