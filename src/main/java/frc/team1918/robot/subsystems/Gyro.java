package frc.team1918.robot.subsystems;

import java.util.Map;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.team1918.robot.Constants;

public class Gyro implements Sendable {
	private static Gyro instance;

	private static AHRS m_gyro = new AHRS(SPI.Port.kMXP);
    private static double yawOffset = 0;

	/**
	 * Returns the instance of the Gyro subsystem.
	 * The purpose of this is to only create an instance if one does not already exist.
	 * @return GyroSubSystem instance
	 */
    public static Gyro getInstance() {
		if (instance == null)
			instance = new Gyro();
		return instance;
	}

	/**
	 * Initializes the GyroSubsystem class, performs setup steps, etc.
	 */
    public Gyro() {
    }

	public void buildDashboards() {
		if(Constants.Lighting.debugDashboard) {
			ShuffleboardTab gyroTab = Shuffleboard.getTab("Debug: Gyro");
			gyroTab.add("Value", this)
				.withSize(5, 4)
				.withPosition(0, 0)  
				.withProperties(Map.of("counter_clockwise_positive",true));
			gyroTab.addNumber("Pitch", this::getPitch)
				.withSize(5, 2)
				.withPosition(0, 4)
				.withWidget("Text Display");
		}
	}

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.setSmartDashboardType("Gyro");
		builder.setActuator(false);
		builder.addDoubleProperty("Value", () -> getHeading().getDegrees(), null);
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
		calc_yaw *= (Constants.Gyro.kGyroReversed ? -1.0 : 1.0);
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
