package frc.team1918.robot.commands.stove;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team1918.robot.subsystems.StoveSubsystem;
import frc.team1918.robot.Helpers;
import frc.team1918.robot.RobotContainer;
import frc.team1918.robot.modules.Burner.BurnerPositions;

public class stove_moveBurnerTo_Conditional extends Command {
  // @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"}) //Dont add "unused" under normal operation
  private final StoveSubsystem m_stove;
  private BurnerPositions m_position = BurnerPositions.HOT;
  private Boolean m_highHeat = true;
  private String m_scoreType = "Pancake";

  /**
   * @param subsystem The subsystem used by this command.
   * @param position The BurnerPosition to move to.
   */
  public stove_moveBurnerTo_Conditional(StoveSubsystem subsystem) {
    m_stove = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(fsr);
  }

  // Allow the command to run while disabled
  @Override
  public boolean runsWhenDisabled() {
    return false;
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_highHeat = RobotContainer.getAutonBurnerHot();
    m_position = (m_highHeat) ? BurnerPositions.HOT : BurnerPositions.COLD;
    m_scoreType = (m_highHeat) ? "High" : "Mid";
    Helpers.Debug.debug("Stove: ("+m_scoreType+"): Burner to "+ m_position.toString());
    m_stove.moveAimerTo(0.0);
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