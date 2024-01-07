package frc.team1918.robot.commands.stove;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team1918.robot.subsystems.FiveSecondRuleSubsystem;
import frc.team1918.robot.subsystems.StoveSubsystem;
import frc.team1918.robot.subsystems.FiveSecondRuleSubsystem.spatulas;
import frc.team1918.robot.modules.Burner.BurnerPositions;
import frc.team1918.robot.modules.Spatula.SpatulaPositions;
public class stove_moveBurnerHome extends Command {
  // @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"}) //Dont add "unused" under normal operation
  private final StoveSubsystem m_stove;
  private final FiveSecondRuleSubsystem m_fsr;

  /**
   * This command moves the hotplate to the home position.  
   * While disabled, this simply sets the position to home so that the robot doesn't attempt to go to other positions when enabled
   * @param subsystem The subsystem used by this command.
   * @param fsr The FiveSecondRule subsystem required by this command
   */
  public stove_moveBurnerHome(StoveSubsystem subsystem, FiveSecondRuleSubsystem fsr) {
    m_stove = subsystem;
    m_fsr = fsr;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(fsr);
  }

  // Allow the command to run while disabled
  @Override
  public boolean runsWhenDisabled() {
    return true;
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
    m_stove.moveBurnerTo(BurnerPositions.HOME);
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