
package frc.team1918.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
//import constants and subsystem
import frc.team1918.robot.Constants;
import frc.team1918.robot.subsystems.DriveSubsystem;


/**
 * A command that runs the drive actions. This passes the OI inputs on to the appropriate drive system (fieldCentricDrive or humanDrive).
 * fieldCentricDrive is simply a call to humanDrive after gyro corrections are made.
 */
public class drive_defaultDrive extends Command {
  private final DriveSubsystem m_drive;
  private final DoubleSupplier m_forward;
  private final DoubleSupplier m_strafe;
  private final DoubleSupplier m_rotation;


  /**
   * Creates a new drive_defaultDrive.
   *
   * @param subsystem The drive subsystem this command wil run on.
   * @param forward The control input for driving forwards/backwards
   * @param strafe The control input for driving sideways
   * @param rotation The control input for turning
   */
  public drive_defaultDrive(DriveSubsystem subsystem, DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotation) {
    m_drive = subsystem;
    m_forward = forward;
    m_strafe = strafe;
    m_rotation = rotation;
    addRequirements(m_drive);
  }

  @Override
  public void execute() {
    if (m_forward.getAsDouble() != 0 || m_strafe.getAsDouble() != 0 || m_rotation.getAsDouble() != 0) {
      //adjust rotation by multiplier, different if moving vs stationary
      // double m_rotation_adjusted = (m_forward.getAsDouble() != 0 || m_strafe.getAsDouble() != 0) 
      //   ? m_rotation.getAsDouble() * Constants.DriveTrain.DT_TURN_MULT_MOVING 
      //   : m_rotation.getAsDouble() * Constants.DriveTrain.DT_TURN_MULT_STATIONARY;
      double m_rotation_adjusted = m_rotation.getAsDouble();
      double m_forward_adjusted = m_forward.getAsDouble();
      double m_strafe_adjusted = m_strafe.getAsDouble();
      m_drive.drivePercentage(m_forward_adjusted, m_strafe_adjusted, m_rotation_adjusted, Constants.DriveTrain.useFieldCentric);
    } else {
      m_drive.brake(false);
    }
  }
}
