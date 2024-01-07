/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team1918.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.Command;
import frc.team1918.robot.subsystems.StoveSubsystem;
import frc.team1918.robot.subsystems.FiveSecondRuleSubsystem.spatulas;
import frc.team1918.robot.subsystems.FiveSecondRuleSubsystem;
import frc.team1918.robot.commands.fivesecondrule.fsr_moveSpatulaTo;
import frc.team1918.robot.commands.helpers.helpers_debugMessage;
import frc.team1918.robot.modules.Spatula.SpatulaPositions;

public class cg_spatulaGriddleToFloor extends SequentialCommandGroup {
  private final FiveSecondRuleSubsystem m_fsr;
  private final StoveSubsystem m_stove;
  
  /**
   * This command group moves the different systems toward their home limits, to zero the systems to the limit switches
   * @param stove Stove Subsystem
   * @param fsr FiveSecondRule Subsystem
  */
  public cg_spatulaGriddleToFloor(StoveSubsystem stove, FiveSecondRuleSubsystem fsr, spatulas spatula) {
    m_stove = stove;
    m_fsr = fsr;
    addRequirements(m_stove, m_fsr);

    /**
     * Creates a parrallel command group with the objects to run at the same time.
     * Can also include complex things like other command groups or parallel command groups
     */
    addCommands(
        //this is a comma separated list of commands, thus, the last one should not have a comma
        new helpers_debugMessage("Running cg_spatulaFloorToGriddle"),
        // new fsr_moveSpatulaTo(m_fsr, m_stove, spatula, SpatulaPositions.MIDDOWN),
        new fsr_moveSpatulaTo(m_fsr, m_stove, spatula, SpatulaPositions.FLOOR)
    );
  }
}