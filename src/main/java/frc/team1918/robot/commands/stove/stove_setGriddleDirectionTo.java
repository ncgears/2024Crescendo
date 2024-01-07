package frc.team1918.robot.commands.stove;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team1918.robot.subsystems.StoveSubsystem;
import frc.team1918.robot.Constants;
import frc.team1918.robot.Helpers;
import frc.team1918.robot.modules.Griddle.GriddleDirections;
public class stove_setGriddleDirectionTo extends Command {
  // @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"}) //Dont add "unused" under normal operation
  private final StoveSubsystem m_stove;
  private final GriddleDirections m_direction;

  /**
   * @param subsystem The subsystem used by this command.
   * @param direction The GriddleDirection to set the griddle to
   */
  public stove_setGriddleDirectionTo(StoveSubsystem subsystem, GriddleDirections direction) {
    m_stove = subsystem;
    m_direction = direction;
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
    Helpers.Debug.debug("Stove: Griddle direction to "+m_direction.toString());
    m_stove.setGriddleDirectionAndSpeed(m_direction, Constants.Stove.Griddle.kSpeed);
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