/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team1918.robot.commands.helpers;

import edu.wpi.first.wpilibj2.command.Command;
import frc.team1918.robot.Helpers;
import frc.team1918.robot.RobotContainer;

/**
 * A command that ...
 */
public class helpers_testCube extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"}) //Dont add "unused" under normal operation
  private String m_message;

  /**
   * @param subsystem The subsystem used by this command.
   */
  public helpers_testCube(String message) {
    m_message = message;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Helpers.Debug.debug(m_message + RobotContainer.getAutonCone());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
