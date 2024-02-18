package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax m_motor;

    public IntakeSubsystem() {
        m_motor = new CANSparkMax(IntakeConstants.intakeMotorId, MotorType.kBrushless);
    }

    public void forward() {
        m_motor.set(1);
    }

    public void backwords() {
        m_motor.set(-1);
    }

    public void stop() {
        m_motor.set(0);
    }
}
