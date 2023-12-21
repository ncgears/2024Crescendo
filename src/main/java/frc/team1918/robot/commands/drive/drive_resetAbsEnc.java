
package frc.team1918.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
//import subsystem
import frc.team1918.robot.subsystems.DriveSubsystem;

/**
 * A command to move all swerve modules to their home positions, as defined by the calibration process.
 */
public class drive_resetAbsEnc extends Command {
  private final DriveSubsystem m_drive;


  /**
   * @param subsystem The drive subsystem this command will run on.
   */
  public drive_resetAbsEnc(DriveSubsystem subsystem) {
    m_drive = subsystem;
    addRequirements(m_drive);
  }

  @Override
  public void end(boolean interrupted) {
    //m_drive.resetAllAbsEnc();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}