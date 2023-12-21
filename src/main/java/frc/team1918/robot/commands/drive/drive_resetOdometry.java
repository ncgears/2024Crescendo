
package frc.team1918.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team1918.robot.Helpers;
//import subsystem
import frc.team1918.robot.subsystems.DriveSubsystem;

/**
 * A command to reset the gyro to 0.
 * This happens when we have the robot in a known orientation to allow us to track the orientation of the robot.
 * @implNote The reset happens during the end method of the command to ensure that it always executes even if the command completes before the gyro finishes executing the command.
 */
public class drive_resetOdometry extends Command {
  private final DriveSubsystem m_drive;
  private Pose2d m_pose;

  /**
   * @param subsystem The drive subsystem this command will run on.
   */
  public drive_resetOdometry(DriveSubsystem subsystem, Pose2d pose) {
    m_drive = subsystem;
    m_pose = pose;
    // addRequirements(m_drive);
  }

  @Override
  public void end(boolean interrupted) {
    Helpers.Debug.debug("Drive: Reset Odometry");
    m_drive.resetDistances();
    m_drive.resetOdometry(m_pose.getRotation().getDegrees(), m_pose);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}