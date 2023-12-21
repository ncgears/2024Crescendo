package frc.team1918.robot.commands.stove;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team1918.robot.subsystems.StoveSubsystem;
import frc.team1918.robot.Helpers;
import frc.team1918.robot.RobotContainer;
import frc.team1918.robot.modules.HotPlate.HotPlatePositions;

public class stove_moveHotPlateScore_Conditional extends Command {
  // @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"}) //Dont add "unused" under normal operation
  private final StoveSubsystem m_stove;
  private HotPlatePositions m_position;
  private Boolean m_withBlueberries;
  private String m_stackType;

  /**
   * @param subsystem The subsystem used by this command.
   * @param position The BurnerPosition to move to.
   */
  public stove_moveHotPlateScore_Conditional(StoveSubsystem subsystem) {
    m_stove = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(fsr);
  }

  // Allow the command to run while disabled
  @Override
  public boolean runsWhenDisabled() {
    return false;
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_withBlueberries = RobotContainer.getAutonCone();
    m_position = (m_withBlueberries) ? HotPlatePositions.DOWN : HotPlatePositions.LEVEL;
    m_stackType = (m_withBlueberries) ? "Waffle" : "Pancake";
    Helpers.Debug.debug("Stove: ("+m_stackType+"): HotPlate to "+ m_position.toString());
    m_stove.moveAimerTo(0.0);
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