package frc.robot.commands;

import java.util.Timer;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class StartShooting extends Command{
    ShooterSubsystem shooter;
    ConveyorSubsystem conveyorSubsystem;

    public StartShooting(ShooterSubsystem shooter, ConveyorSubsystem conveyor){
        addRequirements(shooter, conveyor);
        this.shooter=shooter;
        this.conveyorSubsystem = conveyor;
    }

    public void initialize(){}

    public void execute(){
        shooter.shooter_forward();
    }

    @Override 
    public void end (boolean bra) {
        this.conveyorSubsystem.forward();
        CompletableFuture.delayedExecutor(1, TimeUnit.SECONDS).execute(() -> {
            Commands.runOnce(() -> {
                this.conveyorSubsystem.stop();
                this.shooter.shooter_stop();
            }, this.conveyorSubsystem, this.shooter);
        });
    }

    public boolean isFinished(){
        return shooter.getSpeed() > 610;
    }
}