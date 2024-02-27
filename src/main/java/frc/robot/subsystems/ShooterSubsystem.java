package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax shooter_leader;
    private final CANSparkMax shooter_follower;

    private final CANSparkMax arm_leader;
    private final CANSparkMax arm_follower;

    private final SparkPIDController armPidController;
    private final SparkPIDController shooterPidController;

    private final AbsoluteEncoder arm_encoder;
    private final AbsoluteEncoder shooter_encoder;


    public ShooterSubsystem() {
        // Shooting Motors
        shooter_leader = new CANSparkMax(ShooterConstants.leftFlyWheelMotor, MotorType.kBrushless);
        shooter_follower  = new CANSparkMax(ShooterConstants.rightFlyWheelMotor, MotorType.kBrushless);
        shooter_follower.follow(shooter_leader, true);

        shooter_leader.restoreFactoryDefaults();

        shooterPidController = shooter_leader.getPIDController();

        shooter_encoder = shooter_leader.getAbsoluteEncoder(Type.kDutyCycle);
        shooterPidController.setFeedbackDevice(shooter_encoder);

        shooter_encoder.setPositionConversionFactor(Math.PI * 2);
        shooter_encoder.setVelocityConversionFactor(Math.PI * 2 / 60);

        shooterPidController.setP(ShooterConstants.kShooterP);
        shooterPidController.setI(ShooterConstants.kShooterI);
        shooterPidController.setD(ShooterConstants.kShooterD);

        shooter_leader.burnFlash();
        
        // articulation motor 
        arm_leader = new CANSparkMax(ShooterConstants.articulaitonMotorLeader, MotorType.kBrushless);
        arm_follower  = new CANSparkMax(ShooterConstants.articulaitonMotorFollower, MotorType.kBrushless);
        arm_follower.follow(arm_leader, false);

        arm_leader.restoreFactoryDefaults();

        arm_leader.setInverted(true);

        armPidController = arm_leader.getPIDController();

        arm_encoder = arm_leader.getAbsoluteEncoder(Type.kDutyCycle);
        armPidController.setFeedbackDevice(arm_encoder);

        arm_encoder.setPositionConversionFactor(2 * Math.PI); // set to radians
        arm_encoder.setVelocityConversionFactor(2 * Math.PI / 60); // set movment velocity to radians per second

        arm_encoder.setInverted(ShooterConstants.inverShooterEncoder); // encoder is inverted, I think

        armPidController.setP(ShooterConstants.kArmP);
        armPidController.setI(ShooterConstants.kArmI);
        armPidController.setD(ShooterConstants.kArmD);

        arm_leader.burnFlash();

        SmartDashboard.putNumber("Set Angle", arm_encoder.getPosition());
        SmartDashboard.putNumber("Set P", armPidController.getP());
        SmartDashboard.putNumber("Set I", armPidController.getI());
        SmartDashboard.putNumber("Set D", armPidController.getD());
        
        SmartDashboard.putNumber("Speaker Angle", ShooterConstants.SpeakerAngle);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Flywheel Speed", shooter_encoder.getPosition());
    }

    public void shooter_forward() {
        shooter_leader.set(0.98);
    }

    public void shooter_ampforward(){
        setspeed(0.15);
    }

    public void shooter_backwords() {
        shooter_leader.set(-.15);
    }

    public void shooter_stop() {
        shooter_leader.set(0);
    }

    public void arm_up() {
        arm_leader.set(-0.15);
    }

    public void arm_down() {
        arm_leader.set(.15);
    }

    public void aim_speaker() {
        double angle = SmartDashboard.getNumber("Sppeaker Angle", 0);
        setAngle(MathUtil.clamp(angle, ShooterConstants.minShooterAngle, ShooterConstants.maxShooterAngle));
    }

    public void setAngle(double rad) {
        double angle = MathUtil.clamp(rad, ShooterConstants.minShooterAngle, ShooterConstants.maxShooterAngle);
        armPidController.setReference(angle, ControlType.kPosition);
    }

    public void setspeed(double rotations_velocity) {
        shooterPidController.setReference(rotations_velocity, ControlType.kVelocity);
    }

    public void arm_stop() {
        SmartDashboard.putNumber("Set position to ", arm_encoder.getPosition());
        setAngle(arm_encoder.getPosition());
    }
}