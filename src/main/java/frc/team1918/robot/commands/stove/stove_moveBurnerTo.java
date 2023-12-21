package frc.team1918.robot.commands.stove;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team1918.robot.subsystems.FiveSecondRuleSubsystem;
import frc.team1918.robot.subsystems.StoveSubsystem;
import frc.team1918.robot.subsystems.FiveSecondRuleSubsystem.spatulas;
import frc.team1918.robot.Helpers;
import frc.team1918.robot.modules.Burner.BurnerPositions;
import frc.team1918.robot.modules.Spatula.SpatulaPositions;
public class stove_moveBurnerTo extends Command {
  // @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"}) //Dont add "unused" under normal operation
  private final StoveSubsystem m_stove;
  private final FiveSecondRuleSubsystem m_fsr;
  private final BurnerPositions m_position;

  /**
   * @param subsystem The subsystem used by this command.
   * @param position The BurnerPosition to move to.
   */
  public stove_moveBurnerTo(StoveSubsystem subsystem, FiveSecondRuleSubsystem fsr, BurnerPositions position) {
    m_stove = subsystem;
    m_fsr = fsr;
    m_position = position;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(fsr);
  }

  // Allow the command to run while disabled
  @Override
  public boolean runsWhenDisabled() {
    return false;
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_stove.moveAimerTo(0.0);
    if (m_fsr.getSpatulaPosition(spatulas.LEFT)==SpatulaPositions.GRIDDLE) { //if the spatula is at griddle, we have to move it to clear!
      m_fsr.moveSpatulaTo(spatulas.LEFT, SpatulaPositions.CLEAR);
    }
    if (m_fsr.getSpatulaPosition(spatulas.RIGHT)==SpatulaPositions.GRIDDLE) { //if the spatula is at griddle, we have to move it to clear!
      m_fsr.moveSpatulaTo(spatulas.RIGHT, SpatulaPositions.CLEAR);
    }
    Helpers.Debug.debug("Stove: Burner position to "+m_position.toString());
    m_stove.moveBurnerTo(m_position);
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