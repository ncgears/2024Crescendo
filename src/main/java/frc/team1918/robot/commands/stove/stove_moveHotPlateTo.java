package frc.team1918.robot.commands.stove;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team1918.robot.subsystems.StoveSubsystem;
import frc.team1918.robot.Helpers;
import frc.team1918.robot.modules.HotPlate.HotPlatePositions;
public class stove_moveHotPlateTo extends Command {
  // @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"}) //Dont add "unused" under normal operation
  private final StoveSubsystem m_stove;
  private final HotPlatePositions m_position;

  /**
   * @param subsystem The subsystem used by this command.
   * @param position The GreaseTrapPosition to move to.
   */
  public stove_moveHotPlateTo(StoveSubsystem subsystem, HotPlatePositions position) {
    m_stove = subsystem;
    m_position = position;
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
    Helpers.Debug.debug("Stove: HotPlate position to "+m_position.toString());
    m_stove.moveHotPlateTo(m_position);
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