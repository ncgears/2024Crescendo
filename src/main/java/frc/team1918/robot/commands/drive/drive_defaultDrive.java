
package frc.team1918.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
//import constants and subsystem
import frc.team1918.robot.constants.*; 
// import frc.team1918.robot.Helpers;
import frc.team1918.robot.RobotContainer;
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
  private final PIDController m_thetaController, m_trackingController;
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
    m_forward = forward;
    m_strafe = strafe;
    m_rotation = rotation;

    m_thetaController = new PIDController(
      DriveTrainConstants.thetaController.kP, 
      DriveTrainConstants.thetaController.kI, 
      DriveTrainConstants.thetaController.kD
    );
    m_thetaController.enableContinuousInput(-180, 180);
    m_thetaController.setIZone(DriveTrainConstants.thetaController.kIZone);
    m_thetaController.setTolerance(DriveTrainConstants.thetaController.kToleranceDegrees);
    m_thetaController.setIntegratorRange(-0.5, 0.5);
    m_trackingController = new PIDController(
      DriveTrainConstants.trackingController.kP, 
      DriveTrainConstants.trackingController.kI, 
      DriveTrainConstants.trackingController.kD
    );
    m_trackingController.enableContinuousInput(-180, 180);
    m_trackingController.setIZone(DriveTrainConstants.trackingController.kIZone);
    m_trackingController.setTolerance(DriveTrainConstants.trackingController.kToleranceDegrees);
    m_trackingController.setIntegratorRange(-0.5, 0.5);
    // m_xController = new PIDController(DriveTrain.xController.kP, DriveTrain.xController.kI, DriveTrain.xController.kD);
    // m_yController = new PIDController(DriveTrain.yController.kP, DriveTrain.yController.kI, DriveTrain.yController.kD);

    addRequirements(m_drive);
  }


  @Override
  public void execute() {
    if (m_forward.getAsDouble() != 0 || m_strafe.getAsDouble() != 0 || m_rotation.getAsDouble() != 0 || m_drive.isTrackingTarget()) { //any commanded movements
      double m_rotation_adjusted = m_rotation.getAsDouble();
      if(m_rotation.getAsDouble() != 0) { //turn requested, unlock heading
        m_drive.unlockHeading();
      } else {
        if(!m_drive.getHeadingLocked()) m_drive.lockHeading();
      }
      if(m_drive.isTrackingTarget()) { //if we are tracking a target
        var target = Rotation2d.fromDegrees(m_drive.getTrackingTargetHeading()).rotateBy(new Rotation2d(Math.PI)).getDegrees();
          double adjusted = -m_trackingController.calculate(RobotContainer.gyro.getYaw().getDegrees(), target);
          // RobotContainer.pose.setTrackingReady(m_trackingController.atSetpoint());
          RobotContainer.pose.setTrackingReady((Math.abs(m_trackingController.getPositionError()) <= DriveTrainConstants.trackingController.kToleranceDegrees+1));
          // if(!m_trackingController.atSetpoint()) m_rotation_adjusted = adjusted;
          m_rotation_adjusted = adjusted;
          // Helpers.Debug.debug("m_rotation_adjusted="+m_rotation_adjusted);
      } else if(m_drive.getHeadingLocked()) { //locked heading, calculate adjustment
        if(DriveTrainConstants.thetaController.isEnabled) {
          double adjusted = m_rotation_adjusted;
          // if(!m_thetaController.atSetpoint()) adjusted = -m_thetaController.calculate(RobotContainer.gyro.getYaw().getDegrees(), m_drive.getTargetHeading());
          adjusted = -m_thetaController.calculate(RobotContainer.gyro.getYaw().getDegrees(), m_drive.getTargetHeading());
          // Helpers.Debug.debug("theta g="+RobotContainer.gyro.getYaw().getDegrees()+" t="+m_drive.getTargetHeading()+" a="+adjusted);
          m_rotation_adjusted = adjusted;
        }
      }
      double m_forward_adjusted = m_forward.getAsDouble();
      double m_strafe_adjusted = m_strafe.getAsDouble();
      m_drive.drivePercentage(m_forward_adjusted, m_strafe_adjusted, m_rotation_adjusted, DriveTrainConstants.useFieldCentric);
    } else { //no movement requested
      m_drive.brake(false);
    }
  }
}
