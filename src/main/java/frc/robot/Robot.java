package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private Compressor compressor;
  PneumaticHub pneumaticHub;
  private SendableChooser<String> autChooser;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    // pneumaticHub = new PneumaticHub();
    // compressor = new Compressor(50, PneumaticsModuleType.REVPH);
    // compressor.enableAnalog(115,120);

    autChooser = new SendableChooser<String>();
    
    autChooser.addOption("1 note auto", "one");
    autChooser.setDefaultOption("2 note auto", "two");
    autChooser.setDefaultOption("move back", "curve back");


    SmartDashboard.putData("Choose Auto", autChooser);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    // String selected = autChooser.getSelected();
    
    // switch (selected) {
    //   case "one":
    //     m_robotContainer.getOneNoteAuto().schedule();
    //     break;
    //   case "two":
    //     m_robotContainer.getTwoNoteAuto().schedule();
    //     break;
    //   case "curve back":
    //     m_robotContainer.sideAutoAndMoveBack().schedule();
    //   break;
    //   default:
    //     break;
    // }

  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}
}
