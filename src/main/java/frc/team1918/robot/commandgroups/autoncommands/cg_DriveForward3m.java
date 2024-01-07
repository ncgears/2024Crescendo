/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team1918.robot.commandgroups.autoncommands;

// import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team1918.paths.*;
import frc.team1918.robot.Constants;
import frc.team1918.robot.commands.drive.drive_followTrajectory;
import frc.team1918.robot.commands.helpers.helpers_debugMessage;
import frc.team1918.robot.subsystems.DriveSubsystem;
import frc.team1918.robot.subsystems.GyroSubsystem;
import frc.team1918.robot.subsystems.VisionSubsystem;

@SuppressWarnings("unused")
public class cg_DriveForward3m extends SequentialCommandGroup {
  private final DriveSubsystem m_drive;
  private final GyroSubsystem m_gyro;
  private final VisionSubsystem m_vision;

  public cg_DriveForward3m(DriveSubsystem drive, GyroSubsystem gyro, VisionSubsystem vision) {
    m_drive = drive;
    m_gyro = gyro;
    m_vision = vision;
    addRequirements(m_drive, m_gyro, m_vision);

    addCommands(
        //this is a comma separated list of commands, thus, the last one should not have a comma
        //setup the odometry in a starting position from the center of the field (negative is right/back)
        //rotation is the initial rotation of the robot from the downstream direction
        new helpers_debugMessage("Drive: Follow trajectory 'ThreeMetersForward'"),
        new drive_followTrajectory(m_drive, m_gyro, new ThreeMetersForward()), 
        new helpers_debugMessage("Drive: Done with trajectory")
    );
  }
}