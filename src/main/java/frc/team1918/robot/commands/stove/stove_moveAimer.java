
package frc.team1918.robot.commands.stove;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
//import constants and subsystem
import frc.team1918.robot.Constants;
import frc.team1918.robot.subsystems.StoveSubsystem;


@SuppressWarnings("unused")
public class stove_moveAimer extends Command {
  private final StoveSubsystem m_stove;
  private final DoubleSupplier m_aim;

  /**
   * @param subsystem The stove subsystem this command wil run on.
   * @param aim The amount of aim speed (from -1.0 to 1.0)
   */
  public stove_moveAimer(StoveSubsystem subsystem, DoubleSupplier aim) {
    m_stove = subsystem;
    m_aim = aim;
    // addRequirements(m_stove);
  }

  @Override
  public void execute() {
  }

  @Override
  public void initialize() {
    System.out.println(Math.copySign(Constants.Stove.Aimer.kSpeed, m_aim.getAsDouble()));
    if(m_aim.getAsDouble() != 0.0) { 
      m_stove.moveAimer(Math.copySign(Constants.Stove.Aimer.kSpeed, m_aim.getAsDouble()));
    } else {
      m_stove.moveAimer(0.0);
    }
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
