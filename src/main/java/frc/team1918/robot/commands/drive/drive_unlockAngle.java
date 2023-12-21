
package frc.team1918.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
//import subsystem
import frc.team1918.robot.subsystems.DriveSubsystem;

/**
 * A command to stop the calibration mode.
 * This saves the current absolute positions in the homes file stored on the robot. 
 * This allows the values to survive robot reboots and should be performed after any swerve module maintenance.
 */
public class drive_unlockAngle extends Command {
  private final DriveSubsystem m_drive;


  /**
   * @param subsystem The drive subsystem this command will run on.
   */
  public drive_unlockAngle(DriveSubsystem subsystem) {
    m_drive = subsystem;
    addRequirements(m_drive);
  }

  @Override
  public void execute() {
    m_drive.unlockAngle();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}