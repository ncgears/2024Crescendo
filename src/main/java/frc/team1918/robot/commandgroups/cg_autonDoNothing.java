/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team1918.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team1918.robot.commands.helpers.helpers_debugMessage;
import frc.team1918.robot.subsystems.DriveSubsystem;
import frc.team1918.robot.commandgroups.autoncommands.*;

public class cg_autonDoNothing extends SequentialCommandGroup {
  private final DriveSubsystem m_drive;

  public cg_autonDoNothing(DriveSubsystem drive) {
    m_drive = drive;
    addRequirements(m_drive);

    addCommands(
        //this is a comma separated list of commands, thus, the last one should not have a comma
        //setup the odometry in a starting position from the center of the field (negative is right/back)
        //rotation is the initial rotation of the robot from the downstream direction
        new helpers_debugMessage("Auton: ### Do Nothing ###"),
        new cg_SetOdom180(m_drive),
        new cg_Wait(0.5),
        new helpers_debugMessage("Auton: Done with auton")
    );
  }
}