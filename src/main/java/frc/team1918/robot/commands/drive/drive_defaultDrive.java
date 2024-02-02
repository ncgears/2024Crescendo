
package frc.team1918.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
//import constants and subsystem
import frc.team1918.robot.Constants;
import frc.team1918.robot.RobotContainer;
import frc.team1918.robot.subsystems.DriveSubsystem;


/**
 * A command that runs the drive actions. This passes the OI inputs on to the appropriate drive system (fieldCentricDrive or humanDrive).
 * fieldCentricDrive is simply a call to humanDrive after gyro corrections are made.
 */
public class drive_defaultDrive extends Command {
  private final DriveSubsystem m_drive;
  private final double m_forward;
  private final double m_strafe;
  private final double m_rotation;
  private final PIDController m_thetaController;
  // private final PIDController m_xController, m_yController;


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
    m_forward = forward.getAsDouble();
    m_strafe = strafe.getAsDouble();
    m_rotation = rotation.getAsDouble();

    m_thetaController = new PIDController(
      Constants.DriveTrain.thetaController.kP, 
      Constants.DriveTrain.thetaController.kI, 
      Constants.DriveTrain.thetaController.kD
    );
    m_thetaController.enableContinuousInput(-180, 180);
    m_thetaController.setIZone(Constants.DriveTrain.thetaController.kIZone);
    m_thetaController.setTolerance(Constants.DriveTrain.thetaController.kToleranceDegrees);
    m_thetaController.setIntegratorRange(-0.5, 0.5);
    // m_xController = new PIDController(Constants.DriveTrain.xController.kP, Constants.DriveTrain.xController.kI, Constants.DriveTrain.xController.kD);
    // m_yController = new PIDController(Constants.DriveTrain.yController.kP, Constants.DriveTrain.yController.kI, Constants.DriveTrain.yController.kD);

    addRequirements(m_drive);
  }


  @Override
  public void execute() {
    if (m_forward != 0 || m_strafe != 0 || m_rotation != 0) { //any commanded movements
      double m_rotation_adjusted = m_rotation;
      if(m_rotation != 0) { //turn requested, unlock heading
        m_drive.heading_locked = false;
      } else {
        if(!m_drive.heading_locked) m_drive.target_heading = RobotContainer.gyro.getHeading().getDegrees();
        m_drive.heading_locked = true;
      }
      if(m_drive.heading_locked) { //locked heading, calculate adjustment
        // if(!m_thetaController.atSetpoint()) m_rotation_adjusted = m_thetaController.calculate(RobotContainer.gyro.getHeading().getDegrees(),m_drive.target_heading);
        m_rotation_adjusted = m_thetaController.calculate(RobotContainer.gyro.getHeading().getDegrees(), m_drive.target_heading);
      }
      //adjust rotation by multiplier, different if moving vs stationary
      // double m_rotation_adjusted = (m_forward != 0 || m_strafe != 0) 
      //   ? m_rotation * Constants.DriveTrain.DT_TURN_MULT_MOVING 
      //   : m_rotation * Constants.DriveTrain.DT_TURN_MULT_STATIONARY;
      double m_forward_adjusted = m_forward;
      double m_strafe_adjusted = m_strafe;
      m_drive.drivePercentage(m_forward_adjusted, m_strafe_adjusted, m_rotation_adjusted, Constants.DriveTrain.useFieldCentric);
    } else { //no movement requested
      m_drive.brake(false);
    }
  }
}
