
package frc.team1918.robot.commands.gyro;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team1918.robot.Helpers;
import frc.team1918.robot.RobotContainer;

/**
 * A command to reset the gyro to 0.
 * This happens when we have the robot in a known orientation to allow us to track the orientation of the robot.
 * @implNote The reset happens during the end method of the command to ensure that it always executes even if the command completes before the gyro finishes executing the command.
 */
public class gyro_resetGyro extends Command {

  /**
   * @param subsystem The drive subsystem this command will run on.
   */
  public gyro_resetGyro() {
  }

  @Override
  public void end(boolean interrupted) {
    Helpers.Debug.debug("Gyro: Reset Gyro");
    RobotContainer.gyro.zeroHeading();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}