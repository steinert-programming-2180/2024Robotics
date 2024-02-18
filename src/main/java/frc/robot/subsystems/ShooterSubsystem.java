package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import com.revrobotics.SparkPIDController;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax shooter_leader;
    private final CANSparkMax shooter_follower;
    private final CANSparkMax arm_leader;
    private final CANSparkMax arm_follower;

    private final SparkPIDController pidController;

    private final AbsoluteEncoder encoder;

    public ShooterSubsystem() {
        // Shooting Motors
        shooter_leader = new CANSparkMax(ShooterConstants.leftFlyWheelMotor, MotorType.kBrushless);
        shooter_follower  = new CANSparkMax(ShooterConstants.rightFlyWheelMotor, MotorType.kBrushless);
        shooter_follower.follow(shooter_leader, true);

        arm_leader = new CANSparkMax(ShooterConstants.raiseMotor1, MotorType.kBrushless);
        arm_follower  = new CANSparkMax(ShooterConstants.raiseMotor2, MotorType.kBrushless);
        arm_follower.follow(arm_leader, false);

        pidController = arm_leader.getPIDController();

        encoder = arm_leader.getAbsoluteEncoder(Type.kDutyCycle);
        pidController.setFeedbackDevice(encoder);

        pidController.setP(1);
        pidController.setI(.0005);
        pidController.setD(1);


        SmartDashboard.putNumber("angle", encoder.getPosition());
    }

    public void shooter_forward() {
        shooter_leader.set(-0.98);
    }

    public void shooter_backwords() {
        shooter_leader.set(.2);
    }

    public void shooter_stop() {
        shooter_leader.set(0);
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Angle", encoder.getPosition());
    }

    public void arm_up() {
        arm_leader.set(-0.15);
    }

    public void arm_down() {
        arm_leader.set(.15);
    }

    public void arm_stop() {
        arm_leader.set(0);

    
    }
}
