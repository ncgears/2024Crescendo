
package frc.team1918.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team1918.robot.Constants;
import frc.team1918.robot.RobotContainer;
import frc.team1918.lib.control.PIDController;
import frc.team1918.lib.control.SwerveTrajectory;
import frc.team1918.lib.control.SwerveTrajectory.State;
import frc.team1918.paths.Path;
//import subsystem
import frc.team1918.robot.subsystems.DriveSubsystem;

/**
 * A command to stop the calibration mode.
 * This saves the current absolute positions in the homes file stored on the robot. 
 * This allows the values to survive robot reboots and should be performed after any swerve module maintenance.
 */
public class drive_followTrajectory extends Command {
  private DriveSubsystem m_drive;
  private SwerveTrajectory m_trajectory;
  private PIDController m_xController, m_yController, m_thetaController;
  // private Rotation2d m_offset;
  private double m_lastTime = 0;
  private Timer m_timer = new Timer();
  
  /**
   * @param subsystem The drive subsystem this command will run on.
   */
  public drive_followTrajectory(DriveSubsystem drive, Path path) {
    m_drive = drive;
    addRequirements(m_drive);
    m_trajectory = path.getPath();
  }

  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
    // m_drive.resetOdometry(m_trajectory.getInitialPose());
    // m_offset = (m_drive.getHeading()).minus(m_trajectory.getInitialPose().getRotation()); //may also add vision offset?
    // dont think we need this, since getHeading already accounts for the offset.

    m_xController = new PIDController(Constants.Auton.kPTranslationController, 0, 0);
    m_yController = new PIDController(Constants.Auton.kPTranslationController, 0, 0);
    m_thetaController = new PIDController(Constants.Auton.kPThetaController, 0, 0);
    m_thetaController.setContinuous(true);
    m_thetaController.setInputRange(Math.PI * 2);

    m_lastTime = 0;
  }

  @Override
  public void execute() {
    double time = m_timer.get();
    double dt = time - m_lastTime;
    State refState = m_trajectory.sample(time);
    Pose2d currentPose = RobotContainer.pose.getPose();
    
    m_xController.setReference(refState.pose.getX());
    m_yController.setReference(refState.pose.getY());
    m_thetaController.setReference(refState.pose.getRotation().getRadians());

    double vx = m_xController.calculate(currentPose.getX(), dt) + refState.velocity.x;
    double vy = m_yController.calculate(currentPose.getY(), dt) + refState.velocity.y;
    double omega = -m_thetaController.calculate(currentPose.getRotation().getRadians(), dt) - refState.velocity.z;

    // m_drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, m_drive.getHeading().minus(m_offset)));
    m_drive.driveRelative(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, RobotContainer.gyro.getHeading()));
    m_lastTime = time;
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
    m_drive.brake(false);
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_trajectory.getTotalTime());
  }
}