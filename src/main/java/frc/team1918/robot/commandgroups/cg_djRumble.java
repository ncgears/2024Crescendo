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
import frc.team1918.robot.commands.helpers.helpers_debugMessage;
import frc.team1918.robot.commands.helpers.helpers_setRumble;


public class cg_djRumble extends SequentialCommandGroup {
  
  /**
  */
  public cg_djRumble() {

    /**
     * Creates a sequential command group with the objects to run in sequence.
     * Can also include complex things like other command groups or parallel command groups
     */
    addCommands(
        //this is a comma separated list of commands, thus, the last one should not have a comma
        new helpers_debugMessage("Running djRumble"),
        new helpers_setRumble(true, "dj"),
        new WaitCommand(0.25),
        new helpers_setRumble(false, "dj"),
        new WaitCommand(0.25),
        new helpers_setRumble(true, "dj"),
        new WaitCommand(0.25),
        new helpers_setRumble(false, "dj")
    );
  }
}