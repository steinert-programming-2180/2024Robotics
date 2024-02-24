package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    DoubleSolenoid solenoid;

    public Climber(){
        solenoid=new DoubleSolenoid(PneumaticsModuleType.REVPH, 7, 8);
    }

    public void forward(){
        // System.out.println("penis");
        solenoid.set(Value.kForward);
    }

    public void backward(){
        // System.out.println("cbt");
        solenoid.set(Value.kReverse);
    }
}
