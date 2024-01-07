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
import frc.team1918.robot.subsystems.FiveSecondRuleSubsystem;
import frc.team1918.robot.commands.fivesecondrule.fsr_zeroLeftSpatula;
import frc.team1918.robot.commands.fivesecondrule.fsr_zeroRightSpatula;
import frc.team1918.robot.commands.helpers.helpers_debugMessage;
import frc.team1918.robot.commands.stove.stove_zeroAimer;
import frc.team1918.robot.commands.stove.stove_zeroBurner;
import frc.team1918.robot.commands.stove.stove_zeroGreaseTrap;
import frc.team1918.robot.commands.stove.stove_zeroHotPlate;

public class cg_zeroMovingParts extends ParallelCommandGroup {
  private final FiveSecondRuleSubsystem m_fsr;
  private final StoveSubsystem m_stove;
  
  /**
   * This command group moves the different systems toward their home limits, to zero the systems to the limit switches
   * @param stove Stove Subsystem
   * @param fsr FiveSecondRule Subsystem
  */
  public cg_zeroMovingParts(StoveSubsystem stove, FiveSecondRuleSubsystem fsr) {
    m_stove = stove;
    m_fsr = fsr;
    addRequirements(m_stove, m_fsr);

    /**
     * Creates a parrallel command group with the objects to run at the same time.
     * Can also include complex things like other command groups or parallel command groups
     */
    addCommands(
        //this is a comma separated list of commands, thus, the last one should not have a comma
        new helpers_debugMessage("Running cg_zeroMovingParts"),
        new fsr_zeroLeftSpatula(m_fsr),
        new fsr_zeroRightSpatula(m_fsr),
        new stove_zeroAimer(m_stove),
        new stove_zeroGreaseTrap(m_stove),
        new stove_zeroHotPlate(m_stove),
        new stove_zeroBurner(m_stove)
    );
  }
}