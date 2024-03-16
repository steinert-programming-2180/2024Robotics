package frc.robot.commands;

import java.util.Timer;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.TimeUnit;

import org.ejml.ops.ConvertMatrixData;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.LimelightSubsystem;


public class StartShooting extends Command{
    ShooterSubsystem shooter;
    LimelightSubsystem llight;
    ConveyorSubsystem conveyorSubsystem;

    public StartShooting(ShooterSubsystem shooter, ConveyorSubsystem conveyor, LimelightSubsystem llight){
        addRequirements(shooter, conveyor, llight);
        this.shooter = shooter;
        this.conveyorSubsystem = conveyor;
        this.llight = llight;
    }

    public void initialize(){
        
    }

    public void execute() {
        if (llight.getTx() != 0) {
            shooter.shooter_forward();
        } else {
            end(isFinished());
        }
        
    }

    @Override 
    public void end (boolean bra) {
        //  && conveyorSubsystem.hasNote()
        if (llight.getTx() != 0) {
            this.conveyorSubsystem.forward();
            CompletableFuture.delayedExecutor(1, TimeUnit.SECONDS).execute(() -> {
                this.conveyorSubsystem.stop();
                this.shooter.shooter_stop();
            });
        }

        if (!conveyorSubsystem.hasNote()) {
            conveyorSubsystem.turnOffBlinkin();
        }
    }

    public boolean isFinished(){
        // 
        conveyorSubsystem.greenBlinkin();
        return (shooter.getSpeed() > 610) || llight.getTx() == 0;
        
    }
}