package frc.team1918.robot.commands.stove;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team1918.robot.Constants;
import frc.team1918.robot.modules.HotPlate.HotPlatePositions;
import frc.team1918.robot.subsystems.StoveSubsystem;
public class stove_zeroHotPlate extends Command {
  // @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"}) //Dont add "unused" under normal operation
  private final StoveSubsystem m_stove;

  /**
   * @param subsystem The subsystem used by this command.
   */
  public stove_zeroHotPlate(StoveSubsystem subsystem) {
    m_stove = subsystem;
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
    // Helpers.Debug.debug("zeroHotPlate");
    m_stove.setHotPlateZeroSpeed(Constants.Stove.HotPlate.kZeroSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(m_stove.getHotPlateRevLimit()) m_stove.setHotPlateZeroPos();
    m_stove.moveHotPlateTo(HotPlatePositions.HOME);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}