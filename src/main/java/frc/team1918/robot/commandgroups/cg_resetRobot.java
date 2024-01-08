/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team1918.robot.commandgroups;

// import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team1918.robot.subsystems.VisionSubsystem;
import frc.team1918.robot.commands.helpers.helpers_debugMessage;
import frc.team1918.robot.commands.vision.vision_setLight;

public class cg_resetRobot extends SequentialCommandGroup {
  private final VisionSubsystem m_vision;
  
  /**
   * This command groups issues all the different robot reset items that have to get reset on disable
   * <ol>
   * <li>set whirlygig to down</li>
   * <li>retract intake</li>
   * </ol>
   * <br>
   * @param stove Stove Subsystem
   * @param fsr FiveSecondRule Subsystem
   * @param vision Vision Subsystem
  */
  public cg_resetRobot(VisionSubsystem vision) {
    m_vision = vision;
    // addRequirements(m_stove, m_fsr);

    /**
     * Creates a sequential command group with the objects to run in sequence.
     * Can also include complex things like other command groups or parallel command groups
     */
    addCommands(
        //this is a comma separated list of commands, thus, the last one should not have a comma
        new helpers_debugMessage("Start robot reset sequence"),
        new vision_setLight(m_vision, false),
        new SequentialCommandGroup(
          new WaitCommand(1)
        ),
        new helpers_debugMessage("Finish robot reset sequence")
    );
  }
}