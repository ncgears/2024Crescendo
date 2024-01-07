/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team1918.robot.commandgroups.autoncommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team1918.paths.*;
import frc.team1918.robot.Constants;
import frc.team1918.robot.RobotContainer;
import frc.team1918.robot.commands.drive.drive_followTrajectory;
import frc.team1918.robot.commands.drive.drive_resetOdometry;
import frc.team1918.robot.commands.helpers.helpers_debugMessage;
import frc.team1918.robot.commands.stove.stove_moveBurnerTo;
import frc.team1918.robot.commands.stove.stove_moveBurnerTo_Conditional;
import frc.team1918.robot.commands.stove.stove_moveHotPlateScore_Conditional;
import frc.team1918.robot.commands.stove.stove_moveHotPlateTo;
import frc.team1918.robot.commands.stove.stove_setGriddleDirectionTo;
import frc.team1918.robot.modules.Burner.BurnerPositions;
import frc.team1918.robot.modules.Griddle.GriddleDirections;
import frc.team1918.robot.modules.HotPlate.HotPlatePositions;
import frc.team1918.robot.subsystems.DriveSubsystem;
import frc.team1918.robot.subsystems.FiveSecondRuleSubsystem;
import frc.team1918.robot.subsystems.StoveSubsystem;
import frc.team1918.robot.subsystems.VisionSubsystem;

@SuppressWarnings("unused")
public class cg_ScoreConditional extends SequentialCommandGroup {
  private final DriveSubsystem m_drive;
  private final StoveSubsystem m_stove;
  private final FiveSecondRuleSubsystem m_fsr;
  private final VisionSubsystem m_vision;

  public cg_ScoreConditional(DriveSubsystem drive, StoveSubsystem stove, FiveSecondRuleSubsystem fsr, VisionSubsystem vision) {
    m_drive = drive;
    m_stove = stove;
    m_fsr = fsr;
    m_vision = vision;
    addRequirements(m_drive, m_stove, m_fsr, m_vision);

    addCommands(
        //this is a comma separated list of commands, thus, the last one should not have a comma
        //setup the odometry in a starting position from the center of the field (negative is right/back)
        //rotation is the initial rotation of the robot from the downstream direction
        new ParallelCommandGroup(
          new stove_moveHotPlateScore_Conditional(m_stove),
          new stove_moveBurnerTo_Conditional(m_stove)
        ),
        new cg_Wait(1.0), //known good 1.5
        new stove_setGriddleDirectionTo(m_stove, GriddleDirections.FORWARD),
        new cg_Wait(1.5), //known good 2.5
        new ParallelCommandGroup(
          new stove_setGriddleDirectionTo(m_stove, GriddleDirections.STOP),
          new stove_moveBurnerTo(m_stove, m_fsr, BurnerPositions.HOME),
          new stove_moveHotPlateTo(m_stove, HotPlatePositions.HOME)
        ),
        new cg_Wait(0.5)
    );
  }
}