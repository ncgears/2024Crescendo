package frc.team1918.robot.commands.fivesecondrule;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team1918.robot.subsystems.FiveSecondRuleSubsystem;
import frc.team1918.robot.subsystems.StoveSubsystem;
import frc.team1918.robot.subsystems.FiveSecondRuleSubsystem.spatulas;
import frc.team1918.robot.modules.Burner.BurnerPositions;
import frc.team1918.robot.modules.GreaseTrap.GreaseTrapPositions;
import frc.team1918.robot.modules.HotPlate.HotPlatePositions;
import frc.team1918.robot.modules.Spatula.SpatulaPositions;
public class fsr_moveSpatulaTo extends Command {
  private final FiveSecondRuleSubsystem m_fsr;
  private final StoveSubsystem m_stove;
  private final spatulas m_spatula;
  private final SpatulaPositions m_position;

  /**
   * This command moves the Spatula to the home position.  
   * While disabled, this simply sets the position to home so that the robot doesn't attempt to go to other positions when enabled
   * @param subsystem The FiveSecondRule subsystem required by this command
   */
  public fsr_moveSpatulaTo(FiveSecondRuleSubsystem subsystem, StoveSubsystem stove, spatulas spatula, SpatulaPositions position) {
    m_fsr = subsystem;
    m_stove = stove;
    m_spatula = spatula;
    m_position = position;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(stove);
  }

  // Allow the command to run while disabled
  @Override
  public boolean runsWhenDisabled() {
    return false;
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_position==SpatulaPositions.GRIDDLE) { //Griddle position is requested, so make sure Burner is home!
      m_stove.moveAimerTo(0.0);
      m_stove.moveBurnerTo(BurnerPositions.HOME);
      if(m_stove.getGreaseTrapPosition()==GreaseTrapPositions.HOME) m_stove.moveGreaseTrapTo(GreaseTrapPositions.CLEAR);
      if(m_stove.getHotPlatePosition()==HotPlatePositions.HOME) m_stove.moveHotPlateTo(HotPlatePositions.CLEAR);
    }
    m_fsr.moveSpatulaTo(m_spatula, m_position);
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