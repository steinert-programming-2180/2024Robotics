package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstants;;

public class ConveyorSubsystem extends SubsystemBase {
    private final CANSparkMax m_motor;
    private final DigitalInput sensor;
    public final Spark m_blinkinSpark;

    public ConveyorSubsystem() {
        m_motor = new CANSparkMax(ConveyorConstants.conveyorMotorId, MotorType.kBrushless);
        sensor = new DigitalInput(ConveyorConstants.beamBrakeDioId);
        m_blinkinSpark = new Spark(9);
    }

    public boolean hasNote() {
        return !sensor.get();
    }

    public void flashBlinkin() {
        m_blinkinSpark.set(-0.05);
    }

    public void turnOffBlinkin() {
        m_blinkinSpark.set(0.15);
    }

    public void greenBlinkin() {
        m_blinkinSpark.set(0.77);
    }

    public void forward() {
        m_motor.set(0.35);
    }

    public void fastForward(){
        m_motor.set(1);
    }

    public void slowForward(){
        m_motor.set(0.3);
    }

    public void timedfoward(){
        while (true){
            m_motor.set(0.5);
            if (hasNote() == true){
                m_motor.set(0);
                m_motor.set(0.5);
                Timer.delay(0.05);
                m_motor.set(0);
                break;
            }
        }
    }

    public void backwords() {
        m_motor.set(-.5);
    }

    public void stop() {
        m_motor.set(0);
    }

}
