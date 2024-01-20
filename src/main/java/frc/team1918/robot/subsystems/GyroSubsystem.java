package frc.team1918.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team1918.robot.Constants;
import frc.team1918.robot.Dashboard;
import frc.team1918.robot.Robot;

public class GyroSubsystem extends SubsystemBase {
	private static GyroSubsystem instance;

	private static AHRS m_gyro = new AHRS(SPI.Port.kMXP);
    private static double yawOffset = 0;

	/**
	 * Returns the instance of the Gyro subsystem.
	 * The purpose of this is to only create an instance if one does not already exist.
	 * @return GyroSubSystem instance
	 */
    public static GyroSubsystem getInstance() {
		if (instance == null)
			instance = new GyroSubsystem();
		return instance;
	}

	/**
	 * Initializes the GyroSubsystem class, performs setup steps, etc.
	 */
    public GyroSubsystem() {
        // m_gyro.calibrate(); //Deprecated in 2024?

		//Add this sendable to the Dashboard
		SmartDashboard.putData("Gyro", this);
    }

	@Override
	public void periodic() {
		updateDashboard();
		if(Robot.isSimulation()) {
            //This is for updating simulated data
		}
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		super.initSendable(builder);
		builder.setSmartDashboardType("Gyro");
		builder.setActuator(false);
		builder.addDoubleProperty("Heading", () -> getHeading().getDegrees(), null);
		builder.addDoubleProperty("Pitch", this::getPitch, null);
	}

	/**
     * Returns the heading of the robot.
     * @return the robot's heading as a Rotation2d
     */
	public Rotation2d getHeading() {
		double raw_yaw = m_gyro.getYaw() - (double)yawOffset;  //always subtract the offset
		double calc_yaw = raw_yaw;
		if (0.0 > raw_yaw) { //yaw is negative
			calc_yaw += 360.0;
		}
		calc_yaw *= (Constants.Swerve.kGyroReversed ? -1.0 : 1.0);
		return Rotation2d.fromDegrees(calc_yaw);
	}

    public void zeroHeading() {
		m_gyro.zeroYaw();
		yawOffset = 0;
	}

	public AHRS getGyro() {
        return m_gyro;
	}

    public void setYawOffset(double offset) {
        yawOffset = offset;
    }

   	/**
     * Returns the pitch of the robot.
     * @return the robot's pitch as a double in degrees
     */
	public double getPitch() {
		return m_gyro.getPitch();
	}

   	/**
     * Returns the yaw/heading of the robot.
     * @return the robot's yaw as a double in degrees
     */
	public double getYaw() {
		return m_gyro.getYaw();
	}

   	/**
     * Returns the roll of the robot.
     * @return the robot's roll as a double in degrees
     */
	public double getRoll() {
		return m_gyro.getRoll();
	}

	/**
	 * This handles updating the dashboard with data from this subsystem
	 */
    public void updateDashboard() {
    }

}
