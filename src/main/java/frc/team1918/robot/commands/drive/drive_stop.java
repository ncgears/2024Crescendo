
package frc.team1918.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team1918.robot.Helpers;
//import subsystem
import frc.team1918.robot.subsystems.DriveSubsystem;

/**
 * A command to reset the gyro to 0.
 * This happens when we have the robot in a known orientation to allow us to track the orientation of the robot.
 * @implNote The reset happens during the end method of the command to ensure that it always executes even if the command completes before the gyro finishes executing the command.
 */
public class drive_stop extends Command {
  private final DriveSubsystem m_drive;

  /**
   * @param subsystem The drive subsystem this command will run on.
   */
  public drive_stop(DriveSubsystem subsystem) {
    m_drive = subsystem;
    // addRequirements(m_stove, m_fsr);
  }

  @Override
  public void execute() {
    Helpers.Debug.debug("Drive: Stop");
    m_drive.brake(false);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}