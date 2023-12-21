/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team1918.robot.commandgroups.autoncommands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.team1918.robot.subsystems.DriveSubsystem;
import frc.team1918.robot.subsystems.GyroSubsystem;
import frc.team1918.robot.Constants;
import frc.team1918.robot.commands.drive.drive_autoBalance;
import frc.team1918.robot.commands.drive.drive_stop;
import frc.team1918.robot.commands.helpers.helpers_debugMessage;

public class cg_AutoBalance extends SequentialCommandGroup {
  private final DriveSubsystem m_drive;
  private final GyroSubsystem m_gyro;
  
  /**
   * This command groups balances the robot during auton
  */
  public cg_AutoBalance(DriveSubsystem drive, GyroSubsystem gyro) {
    m_drive = drive;
    m_gyro = gyro;
    // addRequirements(m_drive);

    /**
     * Creates a sequential command group with the objects to run in sequence.
     * Can also include complex things like other command groups or parallel command groups
     */
    addCommands(
        //this is a comma separated list of commands, thus, the last one should not have a comma
        new helpers_debugMessage("Auton: Start robot auto balance sequence"),
        new RepeatCommand(
          new SequentialCommandGroup(
            new ParallelDeadlineGroup(
              new WaitCommand(Constants.Auton.AutoBalance.onTime),
              new helpers_debugMessage("Balance: AutoBalance for "+Constants.Auton.AutoBalance.onTime+"s"),
              new drive_autoBalance(m_drive, m_gyro)
            ),
            new ParallelDeadlineGroup(
              new WaitCommand(Constants.Auton.AutoBalance.offTime),
              new drive_stop(m_drive),
              new helpers_debugMessage("Balance: Waiting for "+Constants.Auton.AutoBalance.onTime+"s")
            ),
            new helpers_debugMessage("Loop auto balance sequence")
          )
        ),
        new helpers_debugMessage("Auton: Finish robot auto balance sequence")
    );
  }
}