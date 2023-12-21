
package frc.team1918.robot.commands.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

//import constants and subsystem
import frc.team1918.robot.Constants;
import frc.team1918.robot.Helpers;
import frc.team1918.robot.subsystems.DriveSubsystem;
import frc.team1918.robot.subsystems.VisionSubsystem;


/**
 * A command that runs the drive actions. This passes the OI inputs on to the appropriate drive system (fieldCentricDrive or humanDrive).
 * fieldCentricDrive is simply a call to humanDrive after gyro corrections are made.
 */
public class vision_findTarget extends Command {
  private final DriveSubsystem m_drive;
  private final VisionSubsystem m_vision;
  private static Double m_forward;
  private static Double m_strafe;
  private static Double m_rotation;



  /**
   * Creates a new drive_defaultDrive.
   *
   * @param drive The drive subsystem required for this command
   * @param vision The vision subsystem
   */
  public vision_findTarget(DriveSubsystem drive, VisionSubsystem vision) {
    m_drive = drive;
    m_vision = vision;
    m_forward = 0.0;
    m_strafe = 0.0;
    m_rotation = 0.0;
    addRequirements(m_drive, m_vision);
  }

  public void initialize() {
    Helpers.Debug.debug("Vision: Start Target Assist");
    // m_drive.setVisionTargeting(true);
    m_vision.setRinglight(Constants.Vision.stateLightOn);
  }

  @Override
  public void execute() {
    m_rotation = m_vision.getVisionTurn();
    SmartDashboard.putNumber("Vision/TurnValue", m_rotation);
    // if(m_rotation != 0) m_drive.unlockAngle(); //unlock angle if rotating
    m_drive.drive(m_forward, m_strafe, m_rotation, Constants.DriveTrain.useFieldCentric);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Helpers.Debug.debug("Vision: Stop Target Assist");
    m_drive.brake(false);
    // m_drive.setVisionTargeting(false);
    m_vision.setRinglight(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (false || m_vision.isTargetAcquired());
  }
}
