
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
public class drive_homeSwerves extends Command {
  private final DriveSubsystem m_drive;

  /**
   * @param subsystem The drive subsystem this command will run on.
   */
  public drive_homeSwerves(DriveSubsystem subsystem) {
    m_drive = subsystem;
    addRequirements(m_drive);
  }

  @Override
  public void initialize() {
    Helpers.Debug.debug("Drive: Home Swerves");
    m_drive.homeSwerves();
  }

  @Override
  public boolean isFinished() {
    return m_drive.swervesAtHome();
  }
}