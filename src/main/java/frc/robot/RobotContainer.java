// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.limelightConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AimAmp;
import frc.robot.commands.AimSpeaker;
import frc.robot.commands.ClimbBackward;
import frc.robot.commands.ClimbForward;
import frc.robot.commands.ConveyorBackward;
import frc.robot.commands.ConveyorForward;
import frc.robot.commands.ConveyorStop;
import frc.robot.commands.IntakeBackward;
import frc.robot.commands.IntakeForward;
import frc.robot.commands.IntakeStop;
import frc.robot.commands.LowerShooter;
import frc.robot.commands.RaiseShooter;
import frc.robot.commands.StartShooting;
import frc.robot.commands.StopShooter;
import frc.robot.commands.StopShooting;
import frc.robot.commands.StoppedConveyorForward;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final ConveyorSubsystem m_conveyor = new ConveyorSubsystem();
  private final Climber m_climber = new Climber();
  private final LimelightSubsystem limelight=new LimelightSubsystem();
  private final AHRS m_gyro = new AHRS();

  //teh robot's commands
  private final ConveyorBackward conveyorBackward = new ConveyorBackward(m_conveyor);
  private final ConveyorForward conveyorForward = new ConveyorForward(m_conveyor);
  private final ConveyorStop conveyorStop = new ConveyorStop(m_conveyor);
  private final StoppedConveyorForward stoppedConveyorForward = new StoppedConveyorForward(m_conveyor);

  private final IntakeForward intakeForward = new IntakeForward(m_intake);
  private final IntakeBackward intakeReverse = new IntakeBackward(m_intake);
  private final IntakeStop intakeStop = new IntakeStop(m_intake);

  private final StartShooting shootCommand = new StartShooting(m_shooter);
  private final StopShooting stopShooting = new StopShooting(m_shooter);

  private final RaiseShooter raiseShooter = new RaiseShooter(m_shooter);
  private final LowerShooter lowerShooter = new LowerShooter(m_shooter);
  private final StopShooter stopShooter = new StopShooter(m_shooter);
  private final AimSpeaker aimSpeaker = new AimSpeaker(m_shooter);
  private final AimAmp aimAmp = new AimAmp(m_shooter);

  private final ClimbForward climbForward=new ClimbForward(m_climber);
  private final ClimbBackward climbBackward=new ClimbBackward(m_climber);
  

  private final SendableChooser<Command> autoChooser;

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(1);
  CommandPS5Controller m_ps5driverController = new CommandPS5Controller(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    
    // Register Named Commands
    NamedCommands.registerCommand("conveyorBackward", conveyorBackward);
    NamedCommands.registerCommand("conveyorForward", conveyorForward);
    NamedCommands.registerCommand("conveyorStop", conveyorStop);

    NamedCommands.registerCommand("intakeForward", intakeForward);
    NamedCommands.registerCommand("intakeReverse", intakeReverse);
    NamedCommands.registerCommand("intakeStop", intakeStop);

    NamedCommands.registerCommand("shootCommand", shootCommand);
    NamedCommands.registerCommand("stopShooting", stopShooting);

    NamedCommands.registerCommand("aimSpeaker", aimSpeaker);
    NamedCommands.registerCommand("aimAmp", aimAmp);
    NamedCommands.registerCommand("raiseShooter", raiseShooter);
    NamedCommands.registerCommand("lowerShooter", lowerShooter);
    NamedCommands.registerCommand("stopShooter", stopShooter);


    // Configure the button bindings
    xBoxConfigureButtonBindings();
    pS5ConfigureButtonBindings();

    // Configure XBox default commands
    // m_robotDrive.setDefaultCommand(
    //     // The left stick controls translation of the robot.
    //     // Turning is controlled by the X axis of the right stick.
    //     new RunCommand(
    //         () -> m_robotDrive.drive(
    //             -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
    //             -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
    //             -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
    //             true, true),
    //         m_robotDrive));
    
    // Configure PS5 default commands
    m_robotDrive.setDefaultCommand(
      // The left stick controls translation of the robot.
      // Turning is controlled by the X axis of the right stick.
    new RunCommand(
        () -> m_robotDrive.drive(
          -MathUtil.applyDeadband(m_ps5driverController.getLeftY(), OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(m_ps5driverController.getLeftX(), OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(m_ps5driverController.getRightX(), OIConstants.kDriveDeadband),
          true, true),
        m_robotDrive));

    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void xBoxConfigureButtonBindings() {

    m_driverController.b().onTrue(intakeForward).onFalse(intakeStop);
    m_driverController.b().onTrue(stoppedConveyorForward).onFalse(conveyorStop);
    m_driverController.a().onTrue(intakeReverse).onFalse(intakeStop);

    m_driverController.rightTrigger(.3).onTrue(shootCommand).onFalse(stopShooting);
    m_driverController.leftTrigger(.3).onTrue(new InstantCommand(() -> m_shooter.shooter_ampforward(), m_shooter)).onFalse(new InstantCommand(() -> m_shooter.shooter_stop()));

    m_driverController.y().onTrue(conveyorForward).onFalse(conveyorStop);
    m_driverController.x().onTrue(conveyorBackward).onFalse(conveyorStop);

    m_driverController.leftBumper().onTrue(raiseShooter).onFalse(new InstantCommand(() -> m_shooter.arm_stop(), m_shooter));
    m_driverController.rightBumper().onTrue(lowerShooter).onFalse(new InstantCommand(() -> m_shooter.arm_stop(), m_shooter));

    m_driverController.povLeft().onTrue(climbForward);
    m_driverController.povRight().onTrue(climbBackward);
    
    m_driverController.povUp().onTrue(new InstantCommand(() -> m_shooter.setAngle(ShooterConstants.getImpericalAngle(-Math.abs(limelightConstants.aprilTagX-limelight.getBotX()))), m_shooter));
    m_driverController.povDown().onTrue(new InstantCommand(() -> m_shooter.aim_speaker(), m_shooter));
  
    //m_driverController.leftTrigger(.3).onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));
  }

  private void pS5ConfigureButtonBindings() {

  //   m_ps5driverController.triangle().onTrue(intakeForward).onFalse(intakeStop);
  //   m_ps5driverController.cross().onTrue(intakeReverse).onFalse(intakeStop);

  //   m_ps5driverController.R2().onTrue(shootCommand).onFalse(stopShooting);

  //   m_ps5driverController.circle().onTrue(conveyorForward).onFalse(conveyorStop);
  //   m_ps5driverController.square().onTrue(conveyorBackward).onFalse(conveyorStop);

  //   m_ps5driverController.R1().onTrue(raiseShooter).toggleOnFalse(new RunCommand(() -> m_shooter.arm_stop(), m_shooter));
  //   m_ps5driverController.L1().onTrue(lowerShooter).toggleOnFalse(new RunCommand(() -> m_shooter.arm_stop(), m_shooter));

  //   m_ps5driverController.povUp().onTrue(climbForward);
  //   m_ps5driverController.povDown().onTrue(climbBackward);
  
  //   m_driverController.povLeft().onTrue(new InstantCommand(() -> m_shooter.setAngle(.85), m_shooter));
  //   m_driverController.povRight().onTrue(new InstantCommand(() -> m_shooter.shooter_ampforward(), m_shooter)).onFalse(new InstantCommand(() -> m_shooter.shooter_stop()));
  //   //m_ps5driverController.povUp().onTrue(new InstantCommand(() -> m_shooter.setAngle(.84), m_shooter));

  
  m_ps5driverController.L2().onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));
  }

  public SequentialCommandGroup lineUp() {
    PIDController XYController=ModuleConstants.PID_CONTROLLER;

    ProfiledPIDController thetaController=ModuleConstants.TPID_CONTROLLER;

    Pose2d robotPose=new Pose2d(limelight.getBotX(), limelight.getBotY(), new Rotation2d(m_gyro.getAngle()));

    TrajectoryConfig trajectoryConfig=new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
    AutoConstants.kMaxAccelerationMetersPerSecondSquared);

    double tagY=limelightConstants.aprilTagY;
    double distance=0;
    
    if(limelight.getBotY()<=tagY){
      distance=Math.abs(0.85-limelight.getBotY());
    }

    if(limelight.getBotY()>tagY){
      distance=-Math.abs(0.85-limelight.getBotY());
    }

    Trajectory trajectory=TrajectoryGenerator.generateTrajectory(robotPose, List.of(
      new Translation2d(0, distance)),
      new Pose2d(limelight.getBotX(), tagY, new Rotation2d(0)), trajectoryConfig
    );
    
    SwerveControllerCommand swerveCommand=new SwerveControllerCommand(
      trajectory,
      limelight::getPose,
      DriveConstants.kDriveKinematics,
      XYController,
      XYController,
      thetaController,
      m_robotDrive::setModuleStates,
      m_robotDrive,
      limelight
    );

    double distanceFromTag=-Math.abs(limelightConstants.aprilTagX-limelight.getBotX());

    double angle=ShooterConstants.getImpericalAngle(distanceFromTag);

    return new SequentialCommandGroup(swerveCommand,
      new InstantCommand(() -> m_robotDrive.resetOdometry(trajectory.getInitialPose())), 
      new InstantCommand(() -> m_shooter.setAngle(angle), m_shooter)
    );
  }
  
  public Command getAutonomousCommand() {
    return new Command() {
    };
  }
}


