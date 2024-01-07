package frc.team1918.robot.commands.stove;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team1918.robot.Helpers;
import frc.team1918.robot.subsystems.StoveSubsystem;
public class stove_setConvectionFan extends Command {
  // @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"}) //Dont add "unused" under normal operation
  private final StoveSubsystem m_stove;
  private final boolean m_enabled;

  /**
   * This command stops the griddle.
   * While disabled, this simply sets the speed to stopped so that the robot doesn't attempt to do things when enabled
   * @param subsystem The subsystem used by this command.
   */
  public stove_setConvectionFan(StoveSubsystem subsystem, boolean enabled) {
    m_stove = subsystem;
    m_enabled = enabled;
    // Use addRequirements() here to declare subsystem dependencies.
    //   addRequirements(subsystem);
  }

  // Allow the command to run while disabled
  @Override
  public boolean runsWhenDisabled() {
    return false;
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Helpers.Debug.debug("Stove: Convection Fan on");
    m_stove.setConvectionFan(m_enabled);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}