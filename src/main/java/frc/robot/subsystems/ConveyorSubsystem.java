package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;;

public class ConveyorSubsystem extends SubsystemBase {
    private final CANSparkMax m_motor;
    
    public ConveyorSubsystem() {
        m_motor = new CANSparkMax(ConveyorConstants.conveyorMotorId, MotorType.kBrushless);
    }

    public void forward() {
        m_motor.set(.5);
    }

    public void backwords() {
        m_motor.set(-.5);
    }

    public void stop() {
        m_motor.set(0);
    }
}
