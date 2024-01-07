package frc.team1918.robot.commands.stove;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team1918.robot.subsystems.StoveSubsystem;
import frc.team1918.robot.modules.HotPlate.HotPlatePositions;
public class stove_moveHotPlateHome extends Command {
  // @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"}) //Dont add "unused" under normal operation
  private final StoveSubsystem m_stove;

  /**
   * This command moves the hotplate to the home position.  
   * While disabled, this simply sets the position to home so that the robot doesn't attempt to go to other positions when enabled
   * @param subsystem The subsystem used by this command.
   */
  public stove_moveHotPlateHome(StoveSubsystem subsystem) {
    m_stove = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    //   addRequirements(subsystem);
  }

  // Allow the command to run while disabled
  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_stove.moveHotPlateTo(HotPlatePositions.HOME);
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