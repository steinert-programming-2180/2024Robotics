package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;

public class LimelightSubsystem extends SubsystemBase {
    NetworkTable limeLightTable;
    NetworkTableEntry m_botPos;
    NetworkTableEntry tx;


    public LimelightSubsystem() {
        limeLightTable = NetworkTableInstance.getDefault().getTable("limelight");
        m_botPos = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose");
        limeLightTable = NetworkTableInstance.getDefault().getTable("limelight");
        tx = limeLightTable.getEntry("tx");
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

    public double getSpeakerAngle() {
        double distance = getDistanceToSpeaker();
        if (distance == 0) return 0;

        return Math.asin(2.159 / distance);
    }

    public double getDistanceToSpeaker() {
        LimelightResults results = LimelightHelpers.getLatestResults("");
        int index = canSeeSpeaker(results);

        if (index == -1) return 0;

        Pose3d target = results.targetingResults.targets_Fiducials[index].getTargetPose_CameraSpace();
        target = target.plus(new Transform3d(new Translation3d(0,0, 2.159 - 1.451102), new Rotation3d()));
        
        return Math.sqrt(Math.pow(target.getX(), 2) + Math.pow(target.getY(), 2) + Math.pow(target.getZ(), 2));
    }

    public int canSeeSpeaker(LimelightResults results) {
        LimelightTarget_Fiducial[] fluids = results.targetingResults.targets_Fiducials;
        for (int i = 0; i < fluids.length; i++) {
            if (fluids[i].fiducialID == 8) return i;
        }

        return -1;
    }

    public Double getTx() {
        return tx.getDouble(0);   
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
