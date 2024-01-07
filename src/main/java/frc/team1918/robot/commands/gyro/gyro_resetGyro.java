
package frc.team1918.robot.commands.gyro;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team1918.robot.Helpers;
//import subsystem
import frc.team1918.robot.subsystems.GyroSubsystem;

/**
 * A command to reset the gyro to 0.
 * This happens when we have the robot in a known orientation to allow us to track the orientation of the robot.
 * @implNote The reset happens during the end method of the command to ensure that it always executes even if the command completes before the gyro finishes executing the command.
 */
public class gyro_resetGyro extends Command {
  private final GyroSubsystem m_gyro;

  /**
   * @param subsystem The drive subsystem this command will run on.
   */
  public gyro_resetGyro(GyroSubsystem gyro) {
    m_gyro = gyro;
    addRequirements(m_gyro);
  }

  @Override
  public void end(boolean interrupted) {
    Helpers.Debug.debug("Gyro: Reset Gyro");
    m_gyro.zeroHeading();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}