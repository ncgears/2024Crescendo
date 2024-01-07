/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team1918.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.Command;
import frc.team1918.robot.subsystems.StoveSubsystem;
import frc.team1918.robot.commands.helpers.helpers_debugMessage;
import frc.team1918.robot.commands.stove.stove_moveGreaseTrapTo;
import frc.team1918.robot.commands.stove.stove_setGriddleDirectionTo;
import frc.team1918.robot.modules.GreaseTrap.GreaseTrapPositions;
import frc.team1918.robot.modules.Griddle.GriddleDirections;

public class cg_autoGreaseTrapFlip extends ParallelCommandGroup {
  private final StoveSubsystem m_stove;
  
  /**
   * This command group moves the different systems toward their home limits, to zero the systems to the limit switches
   * @param stove Stove Subsystem
  */
  public cg_autoGreaseTrapFlip(StoveSubsystem stove) {
    m_stove = stove;
    // addRequirements(m_stove);

    /**
     * Creates a parrallel command group with the objects to run at the same time.
     * Can also include complex things like other command groups or parallel command groups
     */
    addCommands(
        //this is a comma separated list of commands, thus, the last one should not have a comma
        new helpers_debugMessage("Running cg_autoGreaseTrapFlip"),
        new stove_moveGreaseTrapTo(m_stove, GreaseTrapPositions.LEVEL),
        new stove_setGriddleDirectionTo(m_stove, GriddleDirections.REVERSE)
        );
  }
}