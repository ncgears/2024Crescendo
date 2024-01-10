
package frc.team1918.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team1918.robot.Helpers;
//import subsystem
import frc.team1918.robot.subsystems.ShooterSubsystem;

/**
 * A command to set the shooter speed to 0.
 */
public class shooter_stopShooter extends Command {
  private final ShooterSubsystem m_shooter;

  /**
   * @param subsystem The drive subsystem this command will run on.
   */
  public shooter_stopShooter(ShooterSubsystem shooter) {
    m_shooter = shooter;
    addRequirements(m_shooter);
  }

  @Override
  public void end(boolean interrupted) {
    Helpers.Debug.debug("Shooter: Stop Shooter");
    m_shooter.stopShooter();
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}