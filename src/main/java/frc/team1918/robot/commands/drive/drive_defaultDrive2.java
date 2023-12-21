
package frc.team1918.robot.commands.drive;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
//import constants and subsystem
import frc.team1918.robot.Constants;
import frc.team1918.robot.Helpers;
import frc.team1918.robot.subsystems.DriveSubsystem;
// import frc.team1918.robot.subsystems.OrchestraSubsystem;


/**
 * A command that runs the drive actions. This passes the OI inputs on to the appropriate drive system (fieldCentricDrive or humanDrive).
 * fieldCentricDrive is simply a call to humanDrive after gyro corrections are made.
 */
@SuppressWarnings("unused")
public class drive_defaultDrive2 extends Command {
  private final DriveSubsystem m_drive;
  private final Joystick dj;
  private final Double m_forward;
  private final Double m_strafe;
  private final Double m_rotation;
  // private final OrchestraSubsystem m_orchestra = new OrchestraSubsystem();

  /**
   * Creates a new drive_defaultDrive.
   *
   * @param subsystem The drive subsystem this command wil run on.
   * @param dj The driver Joystick
   */
  public drive_defaultDrive2(DriveSubsystem subsystem, Joystick dj) {
    m_drive = subsystem;
    this.dj=dj;
    m_forward = Helpers.OI.getAxisFwdValue(true);
    m_strafe = Helpers.OI.getAxisStrafeValue(true);
    m_rotation = Helpers.OI.getAxisTurnValue(true);
    addRequirements(m_drive);
  }

  @Override
  public void execute() {
    if (m_forward != 0 || m_strafe != 0 || m_rotation != 0) {
      if(m_rotation != 0) m_drive.unlockAngle(); //unlock angle if rotating
      //adjust rotation by multiplier, different if moving vs stationary
      double m_rotation_adjusted = (m_forward != 0 || m_strafe != 0) ? m_rotation * Constants.DriveTrain.DT_TURN_MULT_MOVING : m_rotation * Constants.DriveTrain.DT_TURN_MULT_STATIONARY;
      double m_forward_adjusted = (m_forward * Constants.DriveTrain.DT_FWD_MULT);
      double m_strafe_adjusted = (m_strafe * Constants.DriveTrain.DT_STR_MULT);
      m_drive.drive(m_forward_adjusted, m_strafe_adjusted, m_rotation_adjusted, Constants.DriveTrain.useFieldCentric);
    } else {
      // if (!m_orchestra.getOrchestraPlaying()){
         m_drive.brake(false);
      // }
    }
  }
}
