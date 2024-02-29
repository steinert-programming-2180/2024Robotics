package frc.robot.subsystems;

import org.json.simple.parser.JSONParser;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
    NetworkTable limeLightTable;
    NetworkTableEntry m_botPos;
    NetworkTableEntry json;

    LinearFilter filterX = LinearFilter.movingAverage(10);
    LinearFilter filterY = LinearFilter.movingAverage(10);
    LinearFilter filterTheta = LinearFilter.movingAverage(10);


    public LimelightSubsystem() {
        limeLightTable = NetworkTableInstance.getDefault().getTable("limelight");
        m_botPos = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose");
        // json = NetworkTableInstance.getDefault().getTable("limelight").getEntry("json");
    }

    public Pose2d getPose() {
        return new Pose2d(getBotX(), getBotY(), Rotation2d.fromDegrees(getAngle()));
    }

    public Double getBotX(){
        double[] botPose = m_botPos.getDoubleArray(new double[6]);
        return round(botPose[0], 2);
        // return round(filterX.lastValue(), 2);
    }

    public Double getBotY(){
        double[] botPose = m_botPos.getDoubleArray(new double[6]);
        return round(botPose[1], 2);
        // return round(filterY.lastValue(), 2);
    }

    public Double getAngle() {
        double[] botPose = m_botPos.getDoubleArray(new double[6]);
        return botPose[5];
        // return round(filterTheta.lastValue(), 2);
    }

    public void periodic(){
        // double[] botPose = m_botPos.getDoubleArray(new double[6]);
        // double x = filterX.calculate(botPose[0]), y = filterY.calculate(botPose[1]), theta = filterTheta.calculate(botPose[5]);

        // if (botPose.length != 0) {
            SmartDashboard.putNumber("x bot pose", getBotX());
            SmartDashboard.putNumber("y bot pose", getBotY());

            SmartDashboard.putNumber("theta z bot pose", getAngle());
        // }
    }

    public static double round(double value, int places) {
        if (places < 0) throw new IllegalArgumentException();
        
        long factor = (long) Math.pow(10, places);
        value = value * factor;
        long tmp = Math.round(value);
        return (double) tmp / factor;
    }
}
