package frc.team1918.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
//import constants and subsystem
import frc.team1918.robot.Constants;
import frc.team1918.robot.RobotContainer;
import frc.team1918.robot.subsystems.DriveSubsystem;

/**
 * A command that runs the drive actions. This passes the OI inputs on to the appropriate drive system (fieldCentricDrive or humanDrive).
 * fieldCentricDrive is simply a call to humanDrive after gyro corrections are made.
 */
public class drive_autoBalance extends Command {
    private final DriveSubsystem m_drive;
    private double m_tolerance = Constants.Auton.Balance.kToleranceDegrees; //degrees
    private final PIDController m_balancePID = new PIDController(Constants.Auton.Balance.kP, Constants.Auton.Balance.kI, Constants.Auton.Balance.kD);
    // private double m_pitchAngle = 0.0;
  
    /**
     * Creates a new autoBalance command.
     * This attempts to maintain balance within kToleranceDegrees until the command is ended.
     * If kUseDefensiveLock is true, it will lock the drivetrain when the command ends.
     * @param subsystem The drive subsystem this command wil run on.
     */
    public drive_autoBalance(DriveSubsystem drive) {
      m_drive = drive;
      addRequirements(m_drive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // m_balancePID.enableContinuousInput(-180.0,180.0);
        // m_drive.unlockAngle(); //unlock the drive angle PID
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double m_pitchAngle = RobotContainer.gyro.getPitch();
        if (Math.abs(m_pitchAngle) > m_tolerance) {
            m_drive.drive(m_balancePID.calculate(m_pitchAngle, 0), 0, 0, false);
        }
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // m_drive.lockAngle();
        // m_drive.brake(Constants.Auton.Balance.kUseDefensiveLock);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }    
} 