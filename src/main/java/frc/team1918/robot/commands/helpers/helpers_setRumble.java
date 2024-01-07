
package frc.team1918.robot.commands.helpers;

import edu.wpi.first.wpilibj2.command.Command;
//import constants and subsystem
import frc.team1918.robot.Helpers;

/**
 * A command that runs the drive actions. This passes the OI inputs on to the appropriate drive system (fieldCentricDrive or humanDrive).
 * fieldCentricDrive is simply a call to humanDrive after gyro corrections are made.
 */
public class helpers_setRumble extends Command {
  boolean rumble = false;
  String stick = "dj";

  /**
   * @param rumble true to rumble, false to stop
   * @param stick dj = driver stick, oj = oper stick
   */
  public helpers_setRumble(boolean rumble, String stick) {
    this.stick = stick;
    this.rumble = rumble;
  }

  public void initialize() {
    Helpers.OI.rumble(rumble, stick);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
